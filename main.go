package main

import (
	"context"
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

func maintainAltitude(targetAlt float64, vessel *spacecenter.Vessel, control *spacecenter.Control) {
	pidController := NewPIDController(
		0.6, 0.0, 0.003, targetAlt, 0.01, 0.0, 1.0,
	)

	for {
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
		alt, err := f.SurfaceAltitude()
		if err != nil {
			panic(err)
		}

		throttle := pidController.Control(alt)
		if alt >= targetAlt {
			throttle = 0
		}
		if err := control.SetThrottle(float32(throttle)); err != nil {
			panic(err)
		}

		time.Sleep(10 * time.Millisecond)
	}
}

func main() {
	// Connect to the kRPC server with all default parameters.
	client := krpcgo.DefaultKRPCClient()
	client.Host = "127.0.0.1"
	if err := client.Connect(context.Background()); err != nil {
		panic(err)
	}
	defer client.Close()

	sc := spacecenter.New(client)
	vessel, _ := sc.ActiveVessel()
	control, _ := vessel.Control()

	if err := control.SetSAS(true); err != nil {
		panic(err)
	}
	if err := control.SetRCS(false); err != nil {
		panic(err)
	}
	if _, err := control.ActivateNextStage(); err != nil {
		panic(err)
	}

	maintainAltitude(100.0, vessel, control)
}
