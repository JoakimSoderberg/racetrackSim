#include "Simulator.h"

Simulator::Simulator(double Ts_in)
{
    Ts = Ts_in;

    ControlCommand c;
    simulationModel = new Model(1);

}

void Simulator::step()
{
    ControlCommand c;
    double vRef;

    simulationModel->controller->calcControl(c, vRef);

    simulationModel->state.v = vRef;

    if(simulationModel->controller->referenceControl.empty())
        simulationModel->state.v =  0;

    simulationModel->step(Ts, c.steerAngleCommand, 0);
}
