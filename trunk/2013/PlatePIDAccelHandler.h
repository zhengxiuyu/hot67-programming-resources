#include "WPILib.h"

class PlatePIDAccelHandler: public PIDOutput , public PIDSource
{
    public:
        PlatePIDAccelHandler(Victor* m_Motor1, Victor* m_Motor2, AnalogChannel* m_PlatePot, ADXL345_SPI *m_plateAccel, double m_PlateAccelFiltered);
        void PIDWrite(float output);
        double PIDGet();
    protected:
    private:
        ADXL345_SPI *m_Accel;
        Victor* m_PlateMotor1; //Plate Motor
        Victor* m_PlateMotor2;
        AnalogChannel* m_PlatePot; //Plate Potentiometer
        double m_PlateAccelFiltered;
};
