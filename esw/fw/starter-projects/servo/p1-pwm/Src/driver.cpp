#include "main.h"
#include "servo.hpp"

extern TIM_HandleTypeDef htim1;


[[noreturn]] auto new_main() -> void {

    // TODO: impl

    for ( ;; ) {

    }

}


extern "C" {
    void Main() {
        new_main();
    }
}
