#ifndef IR_TASK_H
#define IR_TASK_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "IRRing.h"


// ============================================================
// IRTask
//
// Responsável por executar o processamento da coroa IR
// em uma FreeRTOS Task.
//
// Fluxo:
//
// Hardware Timer
//       ↓
// Timer ISR
//       ↓
// Task Notification
//       ↓
// IRTask
//       ↓
// IRRing::process()
// ============================================================

class IRTask
{
public:

    // ========================================================
    // Configurações padrão
    // ========================================================

    static constexpr uint32_t DEFAULT_PERIOD_MS = 5;

    static constexpr uint32_t DEFAULT_STACK_SIZE = 4096;

    static constexpr UBaseType_t DEFAULT_PRIORITY = 3;

    static constexpr BaseType_t DEFAULT_CORE = 1;


    // ========================================================
    // Construtor
    // ========================================================

    explicit IRTask(
        IRRing& ring
    );


    // ========================================================
    // Inicia a FreeRTOS Task
    // ========================================================

    bool begin();


    // ========================================================
    // Para a Task
    // ========================================================

    void end();


    // ========================================================
    // Estado
    // ========================================================

    bool isRunning() const;


    // ========================================================
    // Configuração
    // ========================================================

    void setPeriodMs(
        uint32_t periodMs
    );

    uint32_t getPeriodMs() const;


    void setPriority(
        UBaseType_t priority
    );

    UBaseType_t getPriority() const;


    void setCore(
        BaseType_t core
    );

    BaseType_t getCore() const;


    void setStackSize(
        uint32_t stackSize
    );

    uint32_t getStackSize() const;


    // ========================================================
    // Task Handle
    // ========================================================

    TaskHandle_t getHandle() const;


    // ========================================================
    // Estatísticas
    // ========================================================

    uint32_t getExecutionCount() const;


    // ========================================================
    // Acorda manualmente a Task
    //
    // Normalmente ela será acordada pelo Timer ISR.
    // ========================================================

    void notify();


private:

    // ========================================================
    // Referência para a coroa IR
    // ========================================================

    IRRing& _ring;


    // ========================================================
    // Handle da Task
    // ========================================================

    TaskHandle_t _taskHandle;


    // ========================================================
    // Estado
    // ========================================================

    bool _running;


    // ========================================================
    // Configurações da Task
    // ========================================================

    uint32_t _periodMs;

    uint32_t _stackSize;

    UBaseType_t _priority;

    BaseType_t _core;


    // ========================================================
    // Estatísticas
    // ========================================================

    uint32_t _executionCount;


    // ========================================================
    // Função estática usada pelo FreeRTOS
    // ========================================================

    static void taskEntry(
        void* parameter
    );


    // ========================================================
    // Loop interno da Task
    // ========================================================

    void taskLoop();
};

#endif
