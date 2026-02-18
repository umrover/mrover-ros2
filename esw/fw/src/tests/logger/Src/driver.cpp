#include "main.h"

#include <logger.hpp>
#include <serial/uart.hpp>

extern UART_HandleTypeDef hlpuart1;
extern DMA_HandleTypeDef hdma_lpuart1_tx;

namespace mrover {

    static constexpr UART_HandleTypeDef* LPUART = &hlpuart1;
    static constexpr DMA_HandleTypeDef* UART_TX_DMA = &hdma_lpuart1_tx;

    UART lpuart;

    auto get_uart_options() -> UART::Options {
        UART::Options options;
        options.use_dma = true;
        return options;
    }

    [[noreturn]] auto init() -> void {
        lpuart = UART{LPUART, get_uart_options()};

        Logger::init(&lpuart);
        auto const& logger = Logger::instance();
        logger.info("Hello World!");

        for (auto i = 0;; ++i) {
            logger.info("Hello World! for the %uth time", i);
            HAL_Delay(1000);
        }
    }

    auto uart_tx_callback(UART_HandleTypeDef const* huart) -> void {
        if (huart == LPUART) {
            lpuart.handle_tx_complete();
        }
    }

} // namespace mrover

extern "C" {

void Init() {
    mrover::init();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    mrover::uart_tx_callback(huart);
}
}
