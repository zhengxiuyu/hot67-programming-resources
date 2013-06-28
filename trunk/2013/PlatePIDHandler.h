#include "WPILib.h"

class PlatePIDHandler: public PIDOutput , public PIDSource
{
    public:
        PlatePIDHandler(Victor* m_Plate1, Victor* m_Plate2, AnalogChannel* m_PlatePot, ADXL345_SPI *m_plateAccel);
        PlatePIDHandler::PlatePIDHandler(Victor* m_Plate1, Victor* m_Plate2,  AnalogChannel* m_Pot);
        void PIDWrite(float output);
        double PIDGet();
    protected:
    private:
        ADXL345_SPI *m_Accel;
        Victor* m_PlateMotor1; //Plate Motors
        Victor* m_PlateMotor2;
        AnalogChannel* m_PlatePot; //Plate Potentiometer
};
