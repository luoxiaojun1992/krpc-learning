package main

import (
	"context"
	"math"
	"time"

	krpcgo "github.com/atburke/krpc-go"
	"github.com/atburke/krpc-go/spacecenter"
)

type PIDController struct {
	KP         float64
	KI         float64
	KD         float64
	Target     float64
	SumE       float64
	PreE       float64
	DT         float64
	MaxControl float64
	MinControl float64
}

func NewPIDController(
	KP float64, KI float64, KD float64, target float64, DT float64, maxControl float64, minControl float64,
) *PIDController {
	return &PIDController{
		KP:         KP,
		KI:         KI,
		KD:         KD,
		Target:     target,
		DT:         DT,
		MaxControl: maxControl,
		MinControl: minControl,
	}
}

func (pc *PIDController) Control(current float64) float64 {
	e := pc.Target - current

	control := pc.KP*e + pc.KI*(pc.SumE+e*pc.DT) + pc.KD*(e-pc.PreE)/pc.DT

	pc.PreE = e
	pc.SumE += e

	if control > pc.MaxControl {
		return 1.0
	} else if control < pc.MinControl {
		return 0
	} else {
		return control
	}
}

func main() {
	// Connect to the kRPC server with all default parameters.
	client := krpcgo.DefaultKRPCClient()
	client.Host = "192.168.31.90"
	if err := client.Connect(context.Background()); err != nil {
		panic(err)
	}
	defer client.Close()

	sc := spacecenter.New(client)
	vessel, err := sc.ActiveVessel()
	if err != nil {
		panic(err)
	}
	control, err := vessel.Control()
	if err != nil {
		panic(err)
	}
	orbit, err := vessel.Orbit()
	if err != nil {
		panic(err)
	}
	body, err := orbit.Body()
	if err != nil {
		panic(err)
	}
	referenceFrame, err := body.ReferenceFrame()
	if err != nil {
		panic(err)
	}
	f, err := vessel.Flight(referenceFrame)
	if err != nil {
		panic(err)
	}

	if err := control.SetSAS(true); err != nil {
		panic(err)
	}
	if err := control.SetRCS(true); err != nil {
		panic(err)
	}
	if _, err := control.ActivateNextStage(); err != nil {
		panic(err)
	}

	// Climbe to 100m
	climbController := NewPIDController(
		0.6, 0.003, 0.05, 100.0, 0.01, 0.0, 1.0,
	)
	for {
		alt, err := f.SurfaceAltitude()
		if err != nil {
			panic(err)
		}

		if alt >= 100.0 {
			break
		}

		throttle := climbController.Control(alt)
		if err := control.SetThrottle(float32(throttle)); err != nil {
			panic(err)
		}

		time.Sleep(10 * time.Millisecond)
	}

	// Move 30m
	println("Moving")
	var pitch, yaw, roll float64
	startPos, err := vessel.Position(referenceFrame)
	if err != nil {
		panic(err)
	}
	pitchController := NewPIDController(
		0.6, 0.05, 0.1, startPos.A+300.0, 0.01, -10, 10,
	)
	yawController := NewPIDController(
		0.6, 0.05, 0.1, startPos.B, 0.01, -10, 10,
	)
	rollController := NewPIDController(
		0.6, 0.05, 0.1, 0.0, 0.01, -10, 10,
	)
	for {
		alt, err := f.SurfaceAltitude()
		if err != nil {
			panic(err)
		}
		currentPos, err := vessel.Position(referenceFrame)
		if err != nil {
			panic(err)
		}
		pitchOffset := startPos.A + 300.0 - currentPos.A
		yawOffset := startPos.B - currentPos.B
		if math.Abs(pitchOffset) <= 50.0 && math.Abs(yawOffset) <= 50.0 {
			break
		}
		println(pitchOffset, yawOffset)
		currentRoll, err := f.Roll()
		if err != nil {
			panic(err)
		}

		newPitch := pitchController.Control(currentPos.A)
		pitchDelta := newPitch - pitch
		if pitchDelta > 1.0 {
			pitchDelta = 1.0
		} else if pitchDelta < -1.0 {
			pitchDelta = -1.0
		}
		if err := control.SetPitch(float32(pitchDelta)); err != nil {
			panic(err)
		}
		pitch += pitchDelta
		newYaw := yawController.Control(currentPos.B)
		yawDelta := newYaw - yaw
		if yawDelta > 1.0 {
			yawDelta = 1.0
		} else if yawDelta < -1.0 {
			yawDelta = -1.0
		}
		if err := control.SetYaw(float32(yawDelta)); err != nil {
			panic(err)
		}
		yaw += yawDelta

		newRoll := rollController.Control(float64(currentRoll))
		rollDelta := newRoll - roll
		if rollDelta > 1.0 {
			rollDelta = 1.0
		} else if rollDelta < -1.0 {
			rollDelta = -1.0
		}
		if err := control.SetRoll(float32(rollDelta)); err != nil {
			panic(err)
		}
		roll += rollDelta

		throttle := climbController.Control(alt)
		if err := control.SetThrottle(float32(throttle)); err != nil {
			panic(err)
		}

		time.Sleep(10 * time.Millisecond)
	}

	// Landing
	println("Landing")
	landingController := NewPIDController(
		0.6, 0.003, 0.05, -3, 0.01, 0.0, 1.0,
	)
	for {
		alt, err := f.SurfaceAltitude()
		if err != nil {
			panic(err)
		}
		if alt <= 100.0 {
			if err := control.SetGear(true); err != nil {
				panic(err)
			}
		}
		if alt <= 10.0 {
			break
		}
		speed, err := f.VerticalSpeed()
		if err != nil {
			panic(err)
		}
		currentPos, err := vessel.Position(referenceFrame)
		if err != nil {
			panic(err)
		}
		currentRoll, err := f.Roll()
		if err != nil {
			panic(err)
		}

		newPitch := pitchController.Control(currentPos.A)
		pitchDelta := newPitch - pitch
		if pitchDelta > 1 {
			pitchDelta = 1
		} else if pitchDelta < -1 {
			pitchDelta = -1
		}
		if err := control.SetPitch(float32(pitchDelta)); err != nil {
			panic(err)
		}
		pitch += pitchDelta
		newYaw := yawController.Control(currentPos.B)
		yawDelta := newYaw - yaw
		if yawDelta > 1 {
			yawDelta = 1
		} else if yawDelta < -1 {
			yawDelta = -1
		}
		if err := control.SetYaw(float32(yawDelta)); err != nil {
			panic(err)
		}
		yaw += yawDelta

		newRoll := rollController.Control(float64(currentRoll))
		rollDelta := newRoll - roll
		if rollDelta > 1.0 {
			rollDelta = 1.0
		} else if rollDelta < -1.0 {
			rollDelta = -1.0
		}
		if err := control.SetRoll(float32(rollDelta)); err != nil {
			panic(err)
		}
		roll += rollDelta

		throttle := landingController.Control(speed)
		if err := control.SetThrottle(float32(throttle)); err != nil {
			panic(err)
		}

		time.Sleep(10 * time.Millisecond)
	}

	control.SetThrottle(0)
}
