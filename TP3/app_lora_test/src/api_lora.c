#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "api_lora.h"

#define TAG "==> LORA_API"

// Global Stuff
static uint8_t PacketParams[6];
static bool txActive;
static int txLost = 0;
static bool debugPrint;
static int SX126x_TXEN = -1;
static int SX126x_RXEN = -1;


// 🔹 Retardo en MILISEGUNDOS (Funciona antes y después de iniciar el scheduler)
static inline void delay(uint32_t ms) {
    #if defined(configUSE_PREEMPTION)  // Verifica si FreeRTOS está presente
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
            vTaskDelay(pdMS_TO_TICKS(ms));  // Cede CPU si el scheduler está activo
        } else {
            HAL_Delay(ms);  // Usa HAL antes de que el scheduler inicie
        }
    #else
        HAL_Delay(ms);  // Sin RTOS, usa HAL_Delay
    #endif
    }
    

// 🔹 Retardo en MICROSEGUNDOS (Funciona siempre)
static inline void delayMicroseconds(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = (HAL_RCC_GetHCLKFreq() / 1000000) * us;  // Convierte us a ciclos de CPU
    
    while ((DWT->CYCCNT - start) < ticks);
}

// 🔹 Inicialización del DWT (para usar DELAY_US)
static inline void DWT_Delay_Init(void) {
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
}

void LoRaErrorDefault(int error)
{
    if (debugPrint)
    {
        printf("LoRaErrorDefault=%d\r\n", error);
    }
    while (true)
    {
        delay(1);
    }
}

__attribute__((weak, alias("LoRaErrorDefault"))) void LoRaError(int error);

bool spi_write_byte(uint8_t* Dataout, size_t DataLength)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (DataLength > 0)
    {
        status = HAL_SPI_Transmit(&hspi4, Dataout, DataLength, HAL_MAX_DELAY);
    }

    return (status == HAL_OK);
}

bool spi_read_byte(uint8_t* Datain, uint8_t* Dataout, size_t DataLength)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (DataLength > 0)
    {
        status = HAL_SPI_TransmitReceive(&hspi4, Dataout, Datain, DataLength, HAL_MAX_DELAY);
    }

    return (status == HAL_OK);
}

uint8_t spi_transfer(uint8_t address)
{
    uint8_t datain[1];
    uint8_t dataout[1];
    dataout[0] = address;
    // spi_write_byte(dataout, 1 );
    spi_read_byte(datain, dataout, 1);
    return datain[0];
}

int16_t LoRaBegin(uint32_t frequencyInHz, int8_t txPowerInDbm,
                  float tcxoVoltage, bool useRegulatorLDO)
{

    if (txPowerInDbm > 22)
        txPowerInDbm = 22;
    if (txPowerInDbm < -3)
        txPowerInDbm = -3;
        

    Reset();

    uint8_t wk[2];
    SetSyncWord(SX126X_SYNC_WORD_PRIVATE);
    ReadRegister(SX126X_REG_LORA_SYNC_WORD_MSB, wk, 2); // 0x0740
    uint16_t syncWord = (wk[0] << 8) + wk[1];
    printf("%s syncWord=0x%x", TAG, syncWord);

    if (syncWord != SX126X_SYNC_WORD_PUBLIC && syncWord != SX126X_SYNC_WORD_PRIVATE)
    {
         printf("%s SX126x error, maybe no SPI connection", TAG);
         return ERR_INVALID_MODE;
     }

    printf("%s SX126x installed", TAG);
    SetStandby(SX126X_STANDBY_RC);

    SetDio2AsRfSwitchCtrl(true);
    printf("%s tcxoVoltage=%f\r\n", TAG, tcxoVoltage);
    // set TCXO control, if requested
    if (tcxoVoltage > 0.0)
    {
        SetDio3AsTcxoCtrl(
            tcxoVoltage,
            RADIO_TCXO_SETUP_TIME); // Configure the radio to use a TCXO controlled by DIO3
    }

    Calibrate(SX126X_CALIBRATE_IMAGE_ON | SX126X_CALIBRATE_ADC_BULK_P_ON |
              SX126X_CALIBRATE_ADC_BULK_N_ON | SX126X_CALIBRATE_ADC_PULSE_ON |
              SX126X_CALIBRATE_PLL_ON | SX126X_CALIBRATE_RC13M_ON | SX126X_CALIBRATE_RC64K_ON);

    printf("%s useRegulatorLDO=%d", TAG, useRegulatorLDO);
    if (useRegulatorLDO)
    {
        SetRegulatorMode(SX126X_REGULATOR_LDO); // set regulator mode: LDO
    }
    else
    {
        SetRegulatorMode(SX126X_REGULATOR_DC_DC); // set regulator mode: DC-DC
    }

    SetBufferBaseAddress(0, 0);
#if 0
	// SX1261_TRANCEIVER
	SetPaConfig(0x06, 0x00, 0x01, 0x01); // PA Optimal Settings +15 dBm
	// SX1262_TRANCEIVER
	SetPaConfig(0x04, 0x07, 0x00, 0x01); // PA Optimal Settings +22 dBm
	// SX1268_TRANCEIVER
	SetPaConfig(0x04, 0x07, 0x00, 0x01); // PA Optimal Settings +22 dBm
#endif
    SetPaConfig(0x04, 0x07, 0x00, 0x01);               // PA Optimal Settings +22 dBm
    SetOvercurrentProtection(60.0);                    // current max 60mA for the whole device
    SetPowerConfig(txPowerInDbm, SX126X_PA_RAMP_200U); // 0 fuer Empfaenger
    SetRfFrequency(frequencyInHz);
    // SetSyncWord(0x0001);
    return ERR_NONE;
}

void FixInvertedIQ(uint8_t iqConfig)
{
    // fixes IQ configuration for inverted IQ
    // see SX1262/SX1268 datasheet, chapter 15 Known Limitations, section 15.4 for details
    // When exchanging LoRa packets with inverted IQ polarity, some packet losses may be observed
    // for longer packets. Workaround: Bit 2 at address 0x0736 must be set to: “0” when using
    // inverted IQ polarity (see the SetPacketParam(...) command) “1” when using standard IQ
    // polarity

    // read current IQ configuration
    uint8_t iqConfigCurrent = 0;
    ReadRegister(SX126X_REG_IQ_POLARITY_SETUP, &iqConfigCurrent, 1); // 0x0736

    // set correct IQ configuration
    // if(iqConfig == SX126X_LORA_IQ_STANDARD) {
    if (iqConfig == SX126X_LORA_IQ_INVERTED)
    {
        iqConfigCurrent &= 0xFB; // using inverted IQ polarity
    }
    else
    {
        iqConfigCurrent |= 0x04; // using standard IQ polarity
    }

    // update with the new value
    WriteRegister(SX126X_REG_IQ_POLARITY_SETUP, &iqConfigCurrent, 1); // 0x0736
}

void LoRaConfig(uint8_t spreadingFactor, uint8_t bandwidth, uint8_t codingRate,
                uint16_t preambleLength, uint8_t payloadLen, bool crcOn, bool invertIrq)
{
    SetStopRxTimerOnPreambleDetect(false);
    SetLoRaSymbNumTimeout(0);
    SetPacketType(SX126X_PACKET_TYPE_LORA); // SX126x.ModulationParams.PacketType : MODEM_LORA
    uint8_t ldro = 0;                       // LowDataRateOptimize OFF
    SetModulationParams(spreadingFactor, bandwidth, codingRate, ldro);

    PacketParams[0] = (preambleLength >> 8) & 0xFF;
    PacketParams[1] = preambleLength;
    if (payloadLen)
    {
        PacketParams[2] = 0x01; // Fixed length packet (implicit header)
        PacketParams[3] = payloadLen;
    }
    else
    {
        PacketParams[2] = 0x00; // Variable length packet (explicit header)
        PacketParams[3] = 0xFF;
    }

    if (crcOn)
        PacketParams[4] = SX126X_LORA_CRC_ON;
    else
        PacketParams[4] = SX126X_LORA_CRC_OFF;

    if (invertIrq)
        PacketParams[5] = 0x01; // Inverted LoRa I and Q signals setup
    else
        PacketParams[5] = 0x00; // Standard LoRa I and Q signals setup

    // fixes IQ configuration for inverted IQ
    FixInvertedIQ(PacketParams[5]);

    WriteCommand(SX126X_CMD_SET_PACKET_PARAMS, PacketParams, 6); // 0x8C

    // Do not use DIO interruptst
    SetDioIrqParams(SX126X_IRQ_ALL,   // all interrupts enabled
                    SX126X_IRQ_NONE,  // interrupts on DIO1
                    SX126X_IRQ_NONE,  // interrupts on DIO2
                    SX126X_IRQ_NONE); // interrupts on DIO3

    // Receive state no receive timeoout
    SetRx(0xFFFFFF);
}

void LoRaDebugPrint(bool enable) { debugPrint = enable; }

uint8_t LoRaReceive(uint8_t* pData, int16_t len)
{
    uint8_t rxLen = 0;
    uint16_t irqRegs = GetIrqStatus();
    // uint8_t status = GetStatus();

    if (irqRegs & SX126X_IRQ_RX_DONE)
    {
        // ClearIrqStatus(SX126X_IRQ_RX_DONE);
        ClearIrqStatus(SX126X_IRQ_ALL);
        rxLen = ReadBuffer(pData, len);
    }

    return rxLen;
}

bool LoRaSend(uint8_t* pData, int16_t len, uint8_t mode)
{
    uint16_t irqStatus;
    bool rv = false;

    if (txActive == false)
    {
        txActive = true;
        PacketParams[2] = 0x00; // Variable length packet (explicit header)
        PacketParams[3] = len;
        WriteCommand(SX126X_CMD_SET_PACKET_PARAMS, PacketParams, 6); // 0x8C

        // ClearIrqStatus(SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT);
        ClearIrqStatus(SX126X_IRQ_ALL);

        WriteBuffer(pData, len);
        SetTx(500);

        if (mode & SX126x_TXMODE_SYNC)
        {
            irqStatus = GetIrqStatus();
            while ((!(irqStatus & SX126X_IRQ_TX_DONE)) && (!(irqStatus & SX126X_IRQ_TIMEOUT)))
            {
                delay(1);
                irqStatus = GetIrqStatus();
            }
            if (debugPrint)
            {
                printf("%s irqStatus=0x%x\r\n",TAG, irqStatus);
                if (irqStatus & SX126X_IRQ_TX_DONE)
                {
                    printf("%s SX126X_IRQ_TX_DONE\r\n", TAG);
                }
                if (irqStatus & SX126X_IRQ_TIMEOUT)
                {
                    printf("%s SX126X_IRQ_TIMEOUT\r\n", TAG);
                }
            }
            txActive = false;

            SetRx(0xFFFFFF);

            if (irqStatus & SX126X_IRQ_TX_DONE)
            {
                rv = true;
            }
        }
        else
        {
            rv = true;
        }
    }
    if (debugPrint)
    {
        printf("%s Send rv=0x%x\r\n", TAG, rv);
    }
    if (rv == false)
        txLost++;
    return rv;
}

bool ReceiveMode(void)
{
    uint16_t irq;
    bool rv = false;

    if (txActive == false)
    {
        rv = true;
    }
    else
    {
        irq = GetIrqStatus();
        if (irq & (SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT))
        {
            SetRx(0xFFFFFF);
            txActive = false;
            rv = true;
        }
    }

    return rv;
}

void GetPacketStatus(int8_t* rssiPacket, int8_t* snrPacket)
{
    uint8_t buf[4];
    ReadCommand(SX126X_CMD_GET_PACKET_STATUS, buf, 4); // 0x14
    *rssiPacket = (buf[3] >> 1) * -1;
    (buf[2] < 128) ? (*snrPacket = buf[2] >> 2) : (*snrPacket = ((buf[2] - 256) >> 2));
}

void SetTxPower(int8_t txPowerInDbm) { SetPowerConfig(txPowerInDbm, SX126X_PA_RAMP_200U); }

void Reset(void)
{
    delay(10);
    HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_RESET);
    delay(20);
    HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_SET);
    delay(10);
    // ensure BUSY is low (state machine ready)
    WaitForIdle(BUSY_WAIT, "Reset", true);
}

void Wakeup(void) { GetStatus(); }

void SetStandby(uint8_t mode)
{
    uint8_t data = mode;
    WriteCommand(SX126X_CMD_SET_STANDBY, &data, 1); // 0x80
}

uint8_t GetStatus(void)
{
    uint8_t rv;
    ReadCommand(SX126X_CMD_GET_STATUS, &rv, 1); // 0xC0
    return rv;
}

void SetDio3AsTcxoCtrl(float voltage, uint32_t delay)
{
    uint8_t buf[4];

    // buf[0] = tcxoVoltage & 0x07;
    if (fabs(voltage - 1.6) <= 0.001)
    {
        buf[0] = SX126X_DIO3_OUTPUT_1_6;
    }
    else if (fabs(voltage - 1.7) <= 0.001)
    {
        buf[0] = SX126X_DIO3_OUTPUT_1_7;
    }
    else if (fabs(voltage - 1.8) <= 0.001)
    {
        buf[0] = SX126X_DIO3_OUTPUT_1_8;
    }
    else if (fabs(voltage - 2.2) <= 0.001)
    {
        buf[0] = SX126X_DIO3_OUTPUT_2_2;
    }
    else if (fabs(voltage - 2.4) <= 0.001)
    {
        buf[0] = SX126X_DIO3_OUTPUT_2_4;
    }
    else if (fabs(voltage - 2.7) <= 0.001)
    {
        buf[0] = SX126X_DIO3_OUTPUT_2_7;
    }
    else if (fabs(voltage - 3.0) <= 0.001)
    {
        buf[0] = SX126X_DIO3_OUTPUT_3_0;
    }
    else
    {
        buf[0] = SX126X_DIO3_OUTPUT_3_3;
    }

    uint32_t delayValue = (float)delay / 15.625;
    buf[1] = (uint8_t)((delayValue >> 16) & 0xFF);
    buf[2] = (uint8_t)((delayValue >> 8) & 0xFF);
    buf[3] = (uint8_t)(delayValue & 0xFF);

    WriteCommand(SX126X_CMD_SET_DIO3_AS_TCXO_CTRL, buf, 4); // 0x97
}

void Calibrate(uint8_t calibParam)
{
    uint8_t data = calibParam;
    WriteCommand(SX126X_CMD_CALIBRATE, &data, 1); // 0x89
}

void SetDio2AsRfSwitchCtrl(uint8_t enable)
{
    uint8_t data = enable;
    WriteCommand(SX126X_CMD_SET_DIO2_AS_RF_SWITCH_CTRL, &data, 1); // 0x9D
}

void SetRfFrequency(uint32_t frequency)
{
    uint8_t buf[4];
    uint32_t freq = 0;

    CalibrateImage(frequency);

    freq = (uint32_t)((double)frequency / (double)FREQ_STEP);
    buf[0] = (uint8_t)((freq >> 24) & 0xFF);
    buf[1] = (uint8_t)((freq >> 16) & 0xFF);
    buf[2] = (uint8_t)((freq >> 8) & 0xFF);
    buf[3] = (uint8_t)(freq & 0xFF);
    WriteCommand(SX126X_CMD_SET_RF_FREQUENCY, buf, 4); // 0x86
}

void CalibrateImage(uint32_t frequency)
{
    uint8_t calFreq[2];

    if (frequency > 900000000)
    {
        calFreq[0] = 0xE1;
        calFreq[1] = 0xE9;
    }
    else if (frequency > 850000000)
    {
        calFreq[0] = 0xD7;
        calFreq[1] = 0xDB;
    }
    else if (frequency > 770000000)
    {
        calFreq[0] = 0xC1;
        calFreq[1] = 0xC5;
    }
    else if (frequency > 460000000)
    {
        calFreq[0] = 0x75;
        calFreq[1] = 0x81;
    }
    else if (frequency > 425000000)
    {
        calFreq[0] = 0x6B;
        calFreq[1] = 0x6F;
    }
    WriteCommand(SX126X_CMD_CALIBRATE_IMAGE, calFreq, 2); // 0x98
}

void SetRegulatorMode(uint8_t mode)
{
    uint8_t data = mode;
    WriteCommand(SX126X_CMD_SET_REGULATOR_MODE, &data, 1); // 0x96
}

void SetBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress)
{
    uint8_t buf[2];

    buf[0] = txBaseAddress;
    buf[1] = rxBaseAddress;
    WriteCommand(SX126X_CMD_SET_BUFFER_BASE_ADDRESS, buf, 2); // 0x8F
}

void SetPowerConfig(int8_t power, uint8_t rampTime)
{
    uint8_t buf[2];

    if (power > 22)
    {
        power = 22;
    }
    else if (power < -3)
    {
        power = -3;
    }

    buf[0] = power;
    buf[1] = (uint8_t)rampTime;
    WriteCommand(SX126X_CMD_SET_TX_PARAMS, buf, 2); // 0x8E
}

void SetPaConfig(uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut)
{
    uint8_t buf[4];

    buf[0] = paDutyCycle;
    buf[1] = hpMax;
    buf[2] = deviceSel;
    buf[3] = paLut;
    WriteCommand(SX126X_CMD_SET_PA_CONFIG, buf, 4); // 0x95
}

void SetOvercurrentProtection(float currentLimit)
{
    if ((currentLimit >= 0.0) && (currentLimit <= 140.0))
    {
        uint8_t buf[1];
        buf[0] = (uint8_t)(currentLimit / 2.5);
        WriteRegister(SX126X_REG_OCP_CONFIGURATION, buf, 1); // 0x08E7
    }
}

void SetSyncWord(int16_t sync)
{
    uint8_t buf[2];

    buf[0] = (uint8_t)((sync >> 8) & 0x00FF);
    buf[1] = (uint8_t)(sync & 0x00FF);
    WriteRegister(SX126X_REG_LORA_SYNC_WORD_MSB, buf, 2); // 0x0740
}

void SetDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask)
{
    uint8_t buf[8];

    buf[0] = (uint8_t)((irqMask >> 8) & 0x00FF);
    buf[1] = (uint8_t)(irqMask & 0x00FF);
    buf[2] = (uint8_t)((dio1Mask >> 8) & 0x00FF);
    buf[3] = (uint8_t)(dio1Mask & 0x00FF);
    buf[4] = (uint8_t)((dio2Mask >> 8) & 0x00FF);
    buf[5] = (uint8_t)(dio2Mask & 0x00FF);
    buf[6] = (uint8_t)((dio3Mask >> 8) & 0x00FF);
    buf[7] = (uint8_t)(dio3Mask & 0x00FF);
    WriteCommand(SX126X_CMD_SET_DIO_IRQ_PARAMS, buf, 8); // 0x08
}

void SetStopRxTimerOnPreambleDetect(bool enable)
{
    printf("%s SetStopRxTimerOnPreambleDetect enable=%d", TAG, enable);
    // uint8_t data = (uint8_t)enable;
    uint8_t data = 0;
    if (enable)
        data = 1;
    WriteCommand(SX126X_CMD_STOP_TIMER_ON_PREAMBLE, &data, 1); // 0x9F
}

void SetLoRaSymbNumTimeout(uint8_t SymbNum)
{
    uint8_t data = SymbNum;
    WriteCommand(SX126X_CMD_SET_LORA_SYMB_NUM_TIMEOUT, &data, 1); // 0xA0
}

void SetPacketType(uint8_t packetType)
{
    uint8_t data = packetType;
    WriteCommand(SX126X_CMD_SET_PACKET_TYPE, &data, 1); // 0x01
}

void SetModulationParams(uint8_t spreadingFactor, uint8_t bandwidth, uint8_t codingRate,
                         uint8_t lowDataRateOptimize)
{
    uint8_t data[4];
    // currently only LoRa supported
    data[0] = spreadingFactor;
    data[1] = bandwidth;
    data[2] = codingRate;
    data[3] = lowDataRateOptimize;
    WriteCommand(SX126X_CMD_SET_MODULATION_PARAMS, data, 4); // 0x8B
}

void SetCadParams(uint8_t cadSymbolNum, uint8_t cadDetPeak, uint8_t cadDetMin, uint8_t cadExitMode,
                  uint32_t cadTimeout)
{
    uint8_t data[7];
    data[0] = cadSymbolNum;
    data[1] = cadDetPeak;
    data[2] = cadDetMin;
    data[3] = cadExitMode;
    data[4] = (uint8_t)((cadTimeout >> 16) & 0xFF);
    data[5] = (uint8_t)((cadTimeout >> 8) & 0xFF);
    data[6] = (uint8_t)(cadTimeout & 0xFF);
    WriteCommand(SX126X_CMD_SET_CAD_PARAMS, data, 7); // 0x88
}

void SetCad()
{
    uint8_t data = 0;
    WriteCommand(SX126X_CMD_SET_CAD, &data, 0); // 0xC5
}

uint16_t GetIrqStatus(void)
{
    uint8_t data[3];
    ReadCommand(SX126X_CMD_GET_IRQ_STATUS, data, 3); // 0x12
    return (data[1] << 8) | data[2];
}

void ClearIrqStatus(uint16_t irq)
{
    uint8_t buf[2];

    buf[0] = (uint8_t)(((uint16_t)irq >> 8) & 0x00FF);
    buf[1] = (uint8_t)((uint16_t)irq & 0x00FF);
    WriteCommand(SX126X_CMD_CLEAR_IRQ_STATUS, buf, 2); // 0x02
}

void SetRx(uint32_t timeout)
{
    if (debugPrint)
    {
        printf("%s ----- SetRx timeout=%" PRIu32 "\r\n", TAG, timeout);
    }
    SetStandby(SX126X_STANDBY_RC);
    SetRxEnable();
    uint8_t buf[3];
    buf[0] = (uint8_t)((timeout >> 16) & 0xFF);
    buf[1] = (uint8_t)((timeout >> 8) & 0xFF);
    buf[2] = (uint8_t)(timeout & 0xFF);

    //WaitForIdle(BUSY_WAIT, "start setrx", true);

    WriteCommand(SX126X_CMD_SET_RX, buf, 3); // 0x82


    for (int retry = 0; retry < 10; retry++)
    {
        if ((GetStatus() & 0x70) == 0x50)
            break;
        delay(1);
    }
    if ((GetStatus() & 0x70) != 0x50)
    {
        printf("%s SetRx Illegal Status", TAG);
        LoRaError(ERR_INVALID_SETRX_STATE);
    }
}

void SetRxEnable(void)
{
    if (debugPrint)
    {
        printf("%s SetRxEnable:SX126x_TXEN=%d SX126x_RXEN=%d\r\n", TAG, SX126x_TXEN, SX126x_RXEN);
    }
    if ((SX126x_TXEN != -1) && (SX126x_RXEN != -1))
    {
        //HAL_GPIO_WritePin(LORA_RXEN_GPIO_Port, LORA_RXEN_Pin, GPIO_PIN_SET);
        //HAL_GPIO_WritePin(LORA_TXEN_GPIO_Port, LORA_TXEN_Pin, GPIO_PIN_RESET);
    }
}

void SetTx(uint32_t timeoutInMs)
{
    if (debugPrint)
    {
        printf("%s ----- SetTx timeoutInMs=%" PRIu32, TAG, timeoutInMs);
    }
    SetStandby(SX126X_STANDBY_RC);
    SetTxEnable();
    uint8_t buf[3];
    uint32_t tout = timeoutInMs;
    if (timeoutInMs != 0)
    {
        uint32_t timeoutInUs = timeoutInMs * 1000;
        tout = (uint32_t)(timeoutInUs / 0.015625);
    }
    if (debugPrint)
    {
        printf("%s SetTx timeoutInMs=%" PRIu32 " tout=%" PRIu32, TAG, timeoutInMs, tout);
    }
    buf[0] = (uint8_t)((tout >> 16) & 0xFF);
    buf[1] = (uint8_t)((tout >> 8) & 0xFF);
    buf[2] = (uint8_t)(tout & 0xFF);
    WriteCommand(SX126X_CMD_SET_TX, buf, 3); // 0x83

    for (int retry = 0; retry < 10; retry++)
    {
        if ((GetStatus() & 0x70) == 0x60)
            break;
        delay(1);
    }
    if ((GetStatus() & 0x70) != 0x60)
    {
        printf("%s SetTx Illegal Status", TAG);
        LoRaError(ERR_INVALID_SETTX_STATE);
    }
}

void SetTxEnable(void)
{
    if (debugPrint)
    {
        printf("%s SetTxEnable:SX126x_TXEN=%d SX126x_RXEN=%d",TAG, SX126x_TXEN, SX126x_RXEN);
    }
    if ((SX126x_TXEN != -1) && (SX126x_RXEN != -1))
    {
        //gpio_set_level(SX126x_RXEN, LOW);
        //gpio_set_level(SX126x_TXEN, HIGH);
    }
}

int GetPacketLost() { return txLost; }

uint8_t GetRssiInst()
{
    uint8_t buf[2];
    ReadCommand(SX126X_CMD_GET_RSSI_INST, buf, 2); // 0x15
    return buf[1];
}

void GetRxBufferStatus(uint8_t* payloadLength, uint8_t* rxStartBufferPointer)
{
    uint8_t buf[3];
    ReadCommand(SX126X_CMD_GET_RX_BUFFER_STATUS, buf, 3); // 0x13
    *payloadLength = buf[1];
    *rxStartBufferPointer = buf[2];
}

void WaitForIdleBegin(unsigned long timeout, char* text)
{
    // ensure BUSY is low (state meachine ready)
    bool stop = false;
    for (int retry = 0; retry < 10; retry++)
    {
        if (retry == 9)
            stop = true;
        bool ret = WaitForIdle(BUSY_WAIT, text, stop);
        if (ret == true)
            break;
        printf("%s WaitForIdle fail retry=%d", TAG, retry);
        delay(1);
    }
}

bool WaitForIdle(unsigned long timeout, char* text, bool stop)
{
    bool ret = true;

#if defined(configUSE_PREEMPTION)  // Si FreeRTOS está presente
    TickType_t start = xTaskGetTickCount();
#else
    uint32_t start = HAL_GetTick();  // Usa HAL_GetTick() en sistemas sin RTOS
#endif

    while (1)
    {
#if defined(configUSE_PREEMPTION)
        if ((xTaskGetTickCount() - start) >= (timeout / portTICK_PERIOD_MS))
            break;
#else
        if ((HAL_GetTick() - start) >= timeout)
            break;
#endif

        if (HAL_GPIO_ReadPin(LORA_BUSY_GPIO_Port, LORA_BUSY_Pin) == GPIO_PIN_RESET)
            break;
    }

    if (HAL_GPIO_ReadPin(LORA_BUSY_GPIO_Port, LORA_BUSY_Pin) == GPIO_PIN_SET)
    {
        printf("%s WaitForIdle Timeout text=%s timeout=%lu start=%lu\n",
               TAG, text, timeout, (unsigned long)start);

        if (stop)
            LoRaError(ERR_IDLE_TIMEOUT);
        else
            ret = false;
    }

    return ret;
}


uint8_t ReadBuffer(uint8_t* rxData, int16_t rxDataLen)
{
    uint8_t offset = 0;
    uint8_t payloadLength = 0;
    GetRxBufferStatus(&payloadLength, &offset);
    if (payloadLength > rxDataLen)
    {
        printf("%s ReadBuffer rxDataLen too small. payloadLength=%d rxDataLen=%d\r\n",
               TAG, payloadLength, rxDataLen);
        return 0;
    }

    // ensure BUSY is low (state meachine ready)
    WaitForIdle(BUSY_WAIT, "start ReadBuffer", true);

    // start transfer
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);

    spi_transfer(SX126X_CMD_READ_BUFFER); // 0x1E
    spi_transfer(offset);
    spi_transfer(SX126X_CMD_NOP);
    for (int i = 0; i < payloadLength; i++)
    {
        rxData[i] = spi_transfer(SX126X_CMD_NOP);
    }

    // stop transfer
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);

    // wait for BUSY to go low
    WaitForIdle(BUSY_WAIT, "end ReadBuffer", false);

    return payloadLength;
}

void WriteBuffer(uint8_t* txData, int16_t txDataLen)
{
    // ensure BUSY is low (state meachine ready)
    WaitForIdle(BUSY_WAIT, "start WriteBuffer", true);

    // start transfer
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);

    spi_transfer(SX126X_CMD_WRITE_BUFFER); // 0x0E
    spi_transfer(0);                       // offset in tx fifo
    for (int i = 0; i < txDataLen; i++)
    {
        spi_transfer(txData[i]);
    }

    // stop transfer
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);

    // wait for BUSY to go low
    WaitForIdle(BUSY_WAIT, "end WriteBuffer", false);
}

void WriteRegister(uint16_t reg, uint8_t* data, uint8_t numBytes)
{
    // ensure BUSY is low (state meachine ready)
    WaitForIdle(BUSY_WAIT, "start WriteRegister", true);

    if (debugPrint)
    {
        printf("%s WriteRegister: REG=0x%02x\r\n", TAG, reg);
    }
    // start transfer
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);

    // send command byte
    spi_transfer(SX126X_CMD_WRITE_REGISTER); // 0x0D
    spi_transfer((reg & 0xFF00) >> 8);
    spi_transfer(reg & 0xff);

    for (uint8_t n = 0; n < numBytes; n++)
    {
        uint8_t in = spi_transfer(data[n]);
        (void)in;
        if (debugPrint)
        {
            printf("%s %02x --> %02x\r\n", TAG, data[n], in);
            // printf("%s DataOut:%02x\r\n", TAG, data[n]);
        }
    }

    // stop transfer
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);

    // wait for BUSY to go low
    WaitForIdle(BUSY_WAIT, "end WriteRegister", false);
#if 0
    if(waitForBusy) {
        WaitForIdle(BUSY_WAIT);
    }
#endif
}

void ReadRegister(uint16_t reg, uint8_t* data, uint8_t numBytes)
{
    // ensure BUSY is low (state meachine ready)
    WaitForIdle(BUSY_WAIT, "start ReadRegister", true);

    if (debugPrint)
    {
        printf("%s ReadRegister: REG=0x%02x\r\n", TAG, reg);
    }

    // start transfer
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);

    // send command byte
    spi_transfer(SX126X_CMD_READ_REGISTER); // 0x1D
    spi_transfer((reg & 0xFF00) >> 8);
    spi_transfer(reg & 0xff);
    spi_transfer(SX126X_CMD_NOP);

    for (uint8_t n = 0; n < numBytes; n++)
    {
        data[n] = spi_transfer(SX126X_CMD_NOP);
        if (debugPrint)
        {
            printf("%s DataIn:%02x\r\n", TAG, data[n]);
        }
    }

    // stop transfer
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);

    // wait for BUSY to go low
    WaitForIdle(BUSY_WAIT, "end ReadRegister", false);
#if 0
    if(waitForBusy) {
        WaitForIdle(BUSY_WAIT);
    }
#endif
}

// WriteCommand with retry
void WriteCommand(uint8_t cmd, uint8_t* data, uint8_t numBytes)
{
    uint8_t status;
    for (int retry = 1; retry < 10; retry++)
    {
        status = WriteCommand2(cmd, data, numBytes);
        printf("%s status=%02x\r\n", TAG, status);
        if (status == 0)
            break;
        printf("%s WriteCommand2 status=%02x retry=%d\r\n", TAG, status, retry);
    }
    if (status != 0)
    {
        printf("%s SPI Transaction error:0x%02x\r\n", TAG, status);
        LoRaError(ERR_SPI_TRANSACTION);
    }
}

uint8_t WriteCommand2(uint8_t cmd, uint8_t* data, uint8_t numBytes)
{
    // ensure BUSY is low (state meachine ready)
    WaitForIdle(BUSY_WAIT, "start WriteCommand2", true);

    // start transfer
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);

    // send command byte
    if (debugPrint)
    {
        printf("%s WriteCommand: CMD=0x%02x\r\n", TAG, cmd);
    }
    spi_transfer(cmd);

    // variable to save error during SPI transfer
    uint8_t status = 0;

    // send/receive all bytes
    for (uint8_t n = 0; n < numBytes; n++)
    {
        uint8_t in = spi_transfer(data[n]);
        if (debugPrint)
        {
            printf("%s %02x --> %02x\r\n", TAG, data[n], in);
        }

        // check status
        if (((in & 0b00001110) == SX126X_STATUS_CMD_TIMEOUT) ||
            ((in & 0b00001110) == SX126X_STATUS_CMD_INVALID) ||
            ((in & 0b00001110) == SX126X_STATUS_CMD_FAILED))
        {
            status = in & 0b00001110;
            break;
        }
        else if (in == 0x00 || in == 0xFF)
        {
            status = SX126X_STATUS_SPI_FAILED;
            break;
        }
    }

    // stop transfer
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);

    // wait for BUSY to go low
    WaitForIdle(BUSY_WAIT, "end WriteCommand2", false);
#if 0
    if(waitForBusy) {
        WaitForIdle(BUSY_WAIT);
    }
#endif

#if 0
    if (status != 0) {
        printf("%s SPI Transaction error:0x%02x\r\n", TAG, status);
        LoRaError(ERR_SPI_TRANSACTION);
    }
#endif
    return status;
}

void ReadCommand(uint8_t cmd, uint8_t* data, uint8_t numBytes)
{
    // ensure BUSY is low (state meachine ready)
    // WaitForIdle(BUSY_WAIT, "start ReadCommand", true);
    WaitForIdleBegin(BUSY_WAIT, "start ReadCommand");

    // start transfer
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);

    // send command byte
    if (debugPrint)
    {
        printf("%s ReadCommand: CMD=0x%02x", TAG, cmd);
    }
    spi_transfer(cmd);

    // send/receive all bytes
    for (uint8_t n = 0; n < numBytes; n++)
    {
        data[n] = spi_transfer(SX126X_CMD_NOP);
        if (debugPrint)
        {
            printf("%s DataIn:%02x", TAG, data[n]);
        }
    }

    // stop transfer
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);

    // wait for BUSY to go low
    delay(1);
    WaitForIdle(BUSY_WAIT, "end ReadCommand", false);
#if 0
	if(waitForBusy) {
		WaitForIdle(BUSY_WAIT);
	}
#endif
}
