#include "IRTask.h"


// ============================================================
// CONSTRUTOR
// ============================================================

IRTask::IRTask(
    IRRing& ring
)
    : _ring(ring)
{
    // --------------------------------------------------------
    // Estado inicial
    // --------------------------------------------------------

    _taskHandle = nullptr;

    _running = false;


    // --------------------------------------------------------
    // Configurações padrão
    // --------------------------------------------------------

    _periodMs =
        DEFAULT_PERIOD_MS;

    _stackSize =
        DEFAULT_STACK_SIZE;

    _priority =
        DEFAULT_PRIORITY;

    _core =
        DEFAULT_CORE;


    // --------------------------------------------------------
    // Estatísticas
    // --------------------------------------------------------

    _executionCount = 0;
}


// ============================================================
// BEGIN
// ============================================================

bool IRTask::begin()
{
    // --------------------------------------------------------
    // Não criar duas Tasks
    // --------------------------------------------------------

    if(_running)
        return true;


    // --------------------------------------------------------
    // Reinicia estatísticas
    // --------------------------------------------------------

    _executionCount = 0;


    // --------------------------------------------------------
    // Configura o período desejado no IRRing
    //
    // O Timer do IRRing trabalha em microssegundos.
    // --------------------------------------------------------

    _ring.setWindowUs(
        _periodMs * 1000UL
    );


    // --------------------------------------------------------
    // Cria a FreeRTOS Task
    // --------------------------------------------------------

    BaseType_t result =
        xTaskCreatePinnedToCore(
            taskEntry,
            "IRTask",
            _stackSize,
            this,
            _priority,
            &_taskHandle,
            _core
        );


    // --------------------------------------------------------
    // Falha na criação
    // --------------------------------------------------------

    if(result != pdPASS)
    {
        _taskHandle =
            nullptr;

        _running =
            false;

        return false;
    }


    // --------------------------------------------------------
    // Task criada
    // --------------------------------------------------------

    _running =
        true;


    // --------------------------------------------------------
    // Entrega o Handle para o IRRing
    //
    // O Timer ISR usará esse Handle para acordar
    // a Task a cada janela.
    // --------------------------------------------------------

    _ring.attachTask(
        _taskHandle
    );


    return true;
}


// ============================================================
// END
// ============================================================

void IRTask::end()
{
    if(!_running)
        return;


    // --------------------------------------------------------
    // Primeiro informa que a Task deve parar
    // --------------------------------------------------------

    _running =
        false;


    // --------------------------------------------------------
    // Acorda a Task caso ela esteja bloqueada
    // em ulTaskNotifyTake().
    // --------------------------------------------------------

    if(_taskHandle != nullptr)
    {
        xTaskNotifyGive(
            _taskHandle
        );
    }


    // --------------------------------------------------------
    // Não fazemos vTaskDelete() aqui imediatamente.
    //
    // A própria Task fará:
    //
    // vTaskDelete(nullptr)
    //
    // ao sair de taskLoop().
    // --------------------------------------------------------

}


// ============================================================
// ESTADO
// ============================================================

bool IRTask::isRunning() const
{
    return _running;
}


// ============================================================
// TASK ENTRY
//
// Essa função é o ponto de entrada exigido pelo FreeRTOS.
// ============================================================

void IRTask::taskEntry(
    void* parameter
)
{
    // --------------------------------------------------------
    // Recupera a instância da classe
    // --------------------------------------------------------

    IRTask* task =
        static_cast<IRTask*>(
            parameter
        );


    // --------------------------------------------------------
    // Proteção contra ponteiro inválido
    // --------------------------------------------------------

    if(task == nullptr)
    {
        vTaskDelete(nullptr);

        return;
    }


    // --------------------------------------------------------
    // Executa o loop principal
    // --------------------------------------------------------

    task->taskLoop();


    // --------------------------------------------------------
    // Task terminou
    // --------------------------------------------------------

    task->_taskHandle =
        nullptr;


    vTaskDelete(nullptr);
}


// ============================================================
// TASK LOOP
//
// A Task fica bloqueada esperando uma notificação.
//
// Quem envia a notificação:
//
// IRRing::timerISR()
//
// através de:
//
// vTaskNotifyGiveFromISR()
// ============================================================

void IRTask::taskLoop()
{
    while(_running)
    {
        // ----------------------------------------------------
        // Espera indefinidamente pela notificação do Timer.
        //
        // pdTRUE:
        // limpa o valor da notificação depois de recebê-la.
        //
        // portMAX_DELAY:
        // espera sem consumir CPU.
        // ----------------------------------------------------

        uint32_t notification =
            ulTaskNotifyTake(
                pdTRUE,
                portMAX_DELAY
            );


        // ----------------------------------------------------
        // Verifica se a Task foi acordada.
        //
        // Normalmente notification será 1.
        // ----------------------------------------------------

        if(notification == 0)
        {
            continue;
        }


        // ----------------------------------------------------
        // Verifica se foi solicitada a parada.
        // ----------------------------------------------------

        if(!_running)
        {
            break;
        }


        // ----------------------------------------------------
        // Processamento da coroa IR
        //
        // Aqui acontece:
        //
        // - snapshot dos contadores
        // - cálculo de intensidade
        // - vetor X
        // - vetor Y
        // - filtro
        // - atan2
        // - ângulo
        // - força
        // - detecção da bola
        // ----------------------------------------------------

        _ring.process();


        // ----------------------------------------------------
        // Estatística
        // ----------------------------------------------------

        _executionCount++;
    }
}


// ============================================================
// CONFIGURAÇÃO DO PERÍODO
// ============================================================

void IRTask::setPeriodMs(
    uint32_t periodMs
)
{
    if(periodMs == 0)
        return;


    _periodMs =
        periodMs;


    // --------------------------------------------------------
    // Atualiza o Timer do IRRing.
    //
    // 5 ms = 5000 us
    // --------------------------------------------------------

    _ring.setWindowUs(
        _periodMs * 1000UL
    );
}


// ============================================================
// GET PERÍODO
// ============================================================

uint32_t IRTask::getPeriodMs() const
{
    return _periodMs;
}


// ============================================================
// PRIORIDADE
// ============================================================

void IRTask::setPriority(
    UBaseType_t priority
)
{
    _priority =
        priority;


    // --------------------------------------------------------
    // Se a Task já existe, atualiza sua prioridade.
    // --------------------------------------------------------

    if(_taskHandle != nullptr)
    {
        vTaskPrioritySet(
            _taskHandle,
            _priority
        );
    }
}


// ============================================================
// GET PRIORIDADE
// ============================================================

UBaseType_t IRTask::getPriority() const
{
    return _priority;
}


// ============================================================
// CORE
// ============================================================

void IRTask::setCore(
    BaseType_t core
)
{
    // --------------------------------------------------------
    // ESP32 clássico possui:
    //
    // Core 0
    // Core 1
    //
    // --------------------------------------------------------

    if(
        core != 0 &&
        core != 1
    )
    {
        return;
    }


    _core =
        core;
}


// ============================================================
// GET CORE
// ============================================================

BaseType_t IRTask::getCore() const
{
    return _core;
}


// ============================================================
// STACK SIZE
// ============================================================

void IRTask::setStackSize(
    uint32_t stackSize
)
{
    if(stackSize == 0)
        return;


    _stackSize =
        stackSize;
}


// ============================================================
// GET STACK SIZE
// ============================================================

uint32_t IRTask::getStackSize() const
{
    return _stackSize;
}


// ============================================================
// GET TASK HANDLE
// ============================================================

TaskHandle_t IRTask::getHandle() const
{
    return _taskHandle;
}


// ============================================================
// EXECUTION COUNT
// ============================================================

uint32_t IRTask::getExecutionCount() const
{
    return _executionCount;
}


// ============================================================
// NOTIFY
//
// Permite acordar manualmente a Task.
//
// Normalmente não será necessário, pois o Timer ISR
// já faz isso.
// ============================================================

void IRTask::notify()
{
    if(_taskHandle == nullptr)
        return;


    xTaskNotifyGive(
        _taskHandle
    );
}
