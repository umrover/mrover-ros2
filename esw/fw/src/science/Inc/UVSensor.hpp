#include <adc.hpp>
#include <cstdint>

namespace mrover {
    class UVSensor {
    private:
        ADCBase* adc;
        uint8_t channel;
        float uv_index{0.0};
        uint16_t adc_res{4095};

    public:
        // default constructor
        UVSensor() = default;

        // adc_in is a pointer to the desired adc and channel_in is the channel that the UVsensor is on
        UVSensor(ADCBase* adc_in, uint8_t const channel_in)
            : adc(adc_in), channel(channel_in) {}

        // sample the sensor by starting a dma transaction
        void sample_sensor() {
            adc->start();
        }

        // update value of uv_index
        float update_uv() {
            uv_index = 33.0 * ((float)adc->get_channel_value(channel) / (float)adc_res);
            return uv_index;
        }

        // returns the current value of uv_index
        float get_current_uv() {
            return uv_index;
        }
    }; // class UVSensor
} // namespace mrover