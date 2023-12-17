package main

import (
	"context"
	"math"
	"os"
	"time"

	krpcgo "github.com/atburke/krpc-go"
	"github.com/atburke/krpc-go/spacecenter"
	"github.com/atburke/krpc-go/types"
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
	KP float64, KI float64, KD float64, target float64, DT float64, minControl, maxControl float64,
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
		return pc.MaxControl
	} else if control < pc.MinControl {
		return pc.MinControl
	} else {
		return control
	}
}

func launchPadReferenceFrame(body *spacecenter.CelestialBody, bodyReferenceFrame *spacecenter.ReferenceFrame) *spacecenter.ReferenceFrame {
	lat := -0.0972
	lon := -74.5577
	temp1, err := bodyReferenceFrame.CreateRelative(types.Tuple3[float64, float64, float64]{},
		types.Tuple4[float64, float64, float64, float64]{A: 0.0, B: math.Sin(-lon / 2.0 * math.Pi / 180.0), C: 0.0, D: math.Cos(-lon / 2.0 * math.Pi / 180.0)},
		types.Tuple3[float64, float64, float64]{},
		types.Tuple3[float64, float64, float64]{},
	)
	if err != nil {
		panic(err)
	}
	temp2, err := temp1.CreateRelative(types.Tuple3[float64, float64, float64]{},
		types.Tuple4[float64, float64, float64, float64]{A: 0.0, B: 0.0, C: math.Sin(lat / 2.0 * math.Pi / 180.0), D: math.Cos(lat / 2.0 * math.Pi / 180.0)},
		types.Tuple3[float64, float64, float64]{},
		types.Tuple3[float64, float64, float64]{},
	)
	if err != nil {
		panic(err)
	}
	height, err := body.SurfaceHeight(lat, lon)
	if err != nil {
		panic(err)
	}
	equatorialRadius, err := body.EquatorialRadius()
	if err != nil {
		panic(err)
	}
	height += float64(equatorialRadius)
	targetFrame, err := temp2.CreateRelative(types.Tuple3[float64, float64, float64]{A: height, B: 0.0, C: 0.0},
		types.Tuple4[float64, float64, float64, float64]{},
		types.Tuple3[float64, float64, float64]{},
		types.Tuple3[float64, float64, float64]{},
	)
	if err != nil {
		panic(err)
	}
	return targetFrame
}

func main() {
	// Connect to the kRPC server with all default parameters.
	client := krpcgo.DefaultKRPCClient()
	client.Host = os.Getenv("KRPC_PORT")
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
	control.SetInputMode(spacecenter.ControlInputMode_Override)
	orbit, err := vessel.Orbit()
	if err != nil {
		panic(err)
	}
	body, err := orbit.Body()
	if err != nil {
		panic(err)
	}
	bodyReferenceFrame, err := body.ReferenceFrame()
	if err != nil {
		panic(err)
	}
	launchPadReferenceFrame := launchPadReferenceFrame(body, bodyReferenceFrame)

	f, err := vessel.Flight(bodyReferenceFrame)
	if err != nil {
		panic(err)
	}

	// surfaceReferenceFrame, err := vessel.SurfaceReferenceFrame()
	// if err != nil {
	// 	panic(err)
	// }
	// craft, err := vessel.Flight(surfaceReferenceFrame)
	// if err != nil {
	// 	panic(err)
	// }

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
		0.6, 0.0, 0.6, 100.0, 0.001, 0.0, 1.0,
	)
	for {
		alt, err := f.SurfaceAltitude()
		if err != nil {
			panic(err)
		}

		if math.Abs(alt-100.0) <= 1.0 {
			break
		}

		throttle := climbController.Control(alt)
		if err := control.SetThrottle(float32(throttle)); err != nil {
			panic(err)
		}

		time.Sleep(1 * time.Millisecond)
	}

	// Move 100m
	println("Moving")
	startPos, err := vessel.Position(launchPadReferenceFrame)
	if err != nil {
		panic(err)
	}
	bController := NewPIDController(
		0.7, 0.3, 0.6, startPos.B, 0.001, -0.5, 0.5,
	)
	cController := NewPIDController(
		0.7, 0.3, 0.6, startPos.C, 0.001, -0.5, 0.5,
	)
	// rollController := NewPIDController(
	// 	0.7, 0.3, 0.6, 0.0, 0.001, -0.5, 0.5,
	// )
	for {
		alt, err := f.SurfaceAltitude()
		if err != nil {
			panic(err)
		}
		currentPos, err := vessel.Position(launchPadReferenceFrame)
		if err != nil {
			panic(err)
		}
		bOffset := startPos.B - currentPos.B
		cOffset := startPos.C - currentPos.C
		if math.Abs(bOffset) <= 5.0 && math.Abs(cOffset) <= 5.0 {
			break
		}
		// currentRoll, err := craft.Roll()
		// if err != nil {
		// 	panic(err)
		// }

		newRight := bController.Control(currentPos.B)
		if err := control.SetRight(float32(newRight)); err != nil {
			panic(err)
		}
		newUp := cController.Control(currentPos.C)
		if err := control.SetUp(-1.0 * float32(newUp)); err != nil {
			panic(err)
		}
		// newRoll := rollController.Control(float64(currentRoll))
		// if err := control.SetRoll(float32(newRoll)); err != nil {
		// 	panic(err)
		// }

		throttle := climbController.Control(alt)
		if err := control.SetThrottle(float32(throttle)); err != nil {
			panic(err)
		}

		time.Sleep(1 * time.Millisecond)
	}

	// Landing
	println("Landing")
	landingController := NewPIDController(
		0.8, 0.01, 0.03, -0.5, 0.001, 0.0, 1.0,
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

		throttle := landingController.Control(speed)
		if err := control.SetThrottle(float32(throttle)); err != nil {
			panic(err)
		}
		println(throttle)

		time.Sleep(1 * time.Millisecond)
	}

	control.SetThrottle(0)
}
