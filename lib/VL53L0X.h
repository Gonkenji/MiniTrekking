#ifndef VL53L0X_h
#define VL53L0X_h

/**
 * \file VL53L0X.h
 * \brief Interface C++ para o sensor de distância VL53L0X (Time-of-Flight) no RP2040.
 *
 * Esta classe provê uma API de alto nível para inicialização, configuração e
 * leitura de distância do sensor VL53L0X através do barramento I2C. O
 * objetivo é facilitar a integração em aplicações RP2040 (Raspberry Pi Pico/Pico W)
 * mantendo compatibilidade com padrões comuns.
 *
 * \author Carlos Delfino
 */

#include <cstdint>

// Forward declaration para evitar dependência direta do header I2C aqui
class I2C;
/**
 * \class VL53L0X
 * \brief Classe que encapsula a comunicação e configuração do sensor VL53L0X.
 *
 * Exemplo típico de uso:
 * \code{.cpp}
 * I2C i2c;
 * i2c.begin();
 * VL53L0X sensor;
 * sensor.setBus(&i2c);
 * if (sensor.init()) {
 *   uint16_t mm = sensor.readRangeSingleMillimeters();
 * }
 * \endcode
 */
class VL53L0X
{
  public:
    // register addresses from API vl53l0x_device.h (ordered as listed there)
    enum regAddr
    {
      SYSRANGE_START                              = 0x00,

      SYSTEM_THRESH_HIGH                          = 0x0C,
      SYSTEM_THRESH_LOW                           = 0x0E,

      SYSTEM_SEQUENCE_CONFIG                      = 0x01,
      SYSTEM_RANGE_CONFIG                         = 0x09,
      SYSTEM_INTERMEASUREMENT_PERIOD              = 0x04,

      SYSTEM_INTERRUPT_CONFIG_GPIO                = 0x0A,

      GPIO_HV_MUX_ACTIVE_HIGH                     = 0x84,

      SYSTEM_INTERRUPT_CLEAR                      = 0x0B,

      RESULT_INTERRUPT_STATUS                     = 0x13,
      RESULT_RANGE_STATUS                         = 0x14,

      RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       = 0xBC,
      RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        = 0xC0,
      RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       = 0xD0,
      RESULT_CORE_RANGING_TOTAL_EVENTS_REF        = 0xD4,
      RESULT_PEAK_SIGNAL_RATE_REF                 = 0xB6,

      ALGO_PART_TO_PART_RANGE_OFFSET_MM           = 0x28,

      I2C_SLAVE_DEVICE_ADDRESS                    = 0x8A,

      MSRC_CONFIG_CONTROL                         = 0x60,

      PRE_RANGE_CONFIG_MIN_SNR                    = 0x27,
      PRE_RANGE_CONFIG_VALID_PHASE_LOW            = 0x56,
      PRE_RANGE_CONFIG_VALID_PHASE_HIGH           = 0x57,
      PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          = 0x64,

      FINAL_RANGE_CONFIG_MIN_SNR                  = 0x67,
      FINAL_RANGE_CONFIG_VALID_PHASE_LOW          = 0x47,
      FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         = 0x48,
      FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,

      PRE_RANGE_CONFIG_SIGMA_THRESH_HI            = 0x61,
      PRE_RANGE_CONFIG_SIGMA_THRESH_LO            = 0x62,

      PRE_RANGE_CONFIG_VCSEL_PERIOD               = 0x50,
      PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          = 0x51,
      PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          = 0x52,

      SYSTEM_HISTOGRAM_BIN                        = 0x81,
      HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       = 0x33,
      HISTOGRAM_CONFIG_READOUT_CTRL               = 0x55,

      FINAL_RANGE_CONFIG_VCSEL_PERIOD             = 0x70,
      FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        = 0x71,
      FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        = 0x72,
      CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       = 0x20,

      MSRC_CONFIG_TIMEOUT_MACROP                  = 0x46,

      SOFT_RESET_GO2_SOFT_RESET_N                 = 0xBF,
      IDENTIFICATION_MODEL_ID                     = 0xC0,
      IDENTIFICATION_REVISION_ID                  = 0xC2,

      OSC_CALIBRATE_VAL                           = 0xF8,

      GLOBAL_CONFIG_VCSEL_WIDTH                   = 0x32,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_0            = 0xB0,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_1            = 0xB1,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_2            = 0xB2,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_3            = 0xB3,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_4            = 0xB4,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_5            = 0xB5,

      GLOBAL_CONFIG_REF_EN_START_SELECT           = 0xB6,
      DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         = 0x4E,
      DYNAMIC_SPAD_REF_EN_START_OFFSET            = 0x4F,
      POWER_MANAGEMENT_GO1_POWER_FORCE            = 0x80,

      VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           = 0x89,

      ALGO_PHASECAL_LIM                           = 0x30,
      ALGO_PHASECAL_CONFIG_TIMEOUT                = 0x30,
    };

    enum vcselPeriodType { VcselPeriodPreRange, VcselPeriodFinalRange };

    uint8_t last_status; // status of last I2C transmission

    /**
     * \brief Construtor padrão. Não inicializa o hardware.
     *
     * Após construir, chame `setBus()` para definir o barramento I2C e
     * `init()` para inicializar o sensor.
     */
    VL53L0X();

    /** \brief Define o barramento I2C a ser utilizado pela instância. */
    void setBus(I2C * bus) { this->bus = bus; }
    /** \brief Retorna o ponteiro para o barramento I2C associado. */
    I2C * getBus() { return bus; }

    /**
     * \brief Altera o endereço I2C do sensor.
     * \param new_addr Novo endereço de 7 bits (0x08..0x77).
     */
    void setAddress(uint8_t new_addr);
    /** \brief Obtém o endereço I2C atual do sensor. */
    inline uint8_t getAddress() { return address; }

    /**
     * \brief Inicializa o sensor e executa a calibração básica.
     * \param io_2v8 Se verdadeiro, configura I/O em 2.8V (padrão). Caso falso, mantém 1.8V.
     * \return `true` em caso de sucesso; `false` se a inicialização falhar.
     */
    bool init(bool io_2v8 = true);

    /** \brief Escreve um registrador de 8 bits. */
    void writeReg(uint8_t reg, uint8_t value);
    /** \brief Escreve um registrador de 16 bits (big-endian). */
    void writeReg16Bit(uint8_t reg, uint16_t value);
    /** \brief Escreve um registrador de 32 bits (big-endian). */
    void writeReg32Bit(uint8_t reg, uint32_t value);
    /** \brief Lê um registrador de 8 bits. */
    uint8_t readReg(uint8_t reg);
    /** \brief Lê um registrador de 16 bits (big-endian). */
    uint16_t readReg16Bit(uint8_t reg);
    /** \brief Lê um registrador de 32 bits (big-endian). */
    uint32_t readReg32Bit(uint8_t reg);

    /**
     * \brief Escreve múltiplos bytes iniciando em um registrador.
     * \param reg Registrador inicial.
     * \param src Ponteiro para o buffer de origem.
     * \param count Quantidade de bytes.
     */
    void writeMulti(uint8_t reg, uint8_t const * src, uint8_t count);
    /**
     * \brief Lê múltiplos bytes a partir de um registrador.
     * \param reg Registrador inicial.
     * \param dst Ponteiro para o buffer de destino.
     * \param count Quantidade de bytes a ler.
     */
    void readMulti(uint8_t reg, uint8_t * dst, uint8_t count);

    /**
     * \brief Define o limite mínimo de taxa de sinal (MCPS).
     * \param limit_Mcps Valor em Mega Counts Per Second (0..511.99).
     * \return `true` em caso de sucesso.
     */
    bool setSignalRateLimit(float limit_Mcps);
    /** \brief Obtém o limite mínimo de taxa de sinal (MCPS). */
    float getSignalRateLimit();

    /**
     * \brief Define o orçamento de tempo de medição (µs).
     * \param budget_us Tempo total destinado a uma medição.
     * \return `true` se aplicado com sucesso.
     */
    bool setMeasurementTimingBudget(uint32_t budget_us);
    /** \brief Retorna o orçamento de tempo de medição atual (µs). */
    uint32_t getMeasurementTimingBudget();

    /**
     * \brief Configura o período do pulso VCSEL (em PCLKs).
     * \param type Tipo do período (pré-range ou final-range).
     * \param period_pclks Valor par válido (ver datasheet).
     * \return `true` em caso de sucesso.
     */
    bool setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks);
    /** \brief Retorna o período do pulso VCSEL (em PCLKs). */
    uint8_t getVcselPulsePeriod(vcselPeriodType type);

    /**
     * \brief Inicia medições contínuas.
     * \param period_ms Período entre medições (ms). Se 0, mede continuamente.
     */
    void startContinuous(uint32_t period_ms = 0);
    /** \brief Interrompe medições contínuas. */
    void stopContinuous();
    /** \brief Lê distância (mm) quando em modo contínuo. */
    uint16_t readRangeContinuousMillimeters();
    /** \brief Realiza uma única leitura de distância (mm). */
    uint16_t readRangeSingleMillimeters();

    /** \brief Define o tempo limite (ms) para operações I2C. */
    inline void setTimeout(uint16_t timeout) { io_timeout = timeout; }
    /** \brief Retorna o tempo limite (ms) configurado. */
    inline uint16_t getTimeout() { return io_timeout; }
    /** \brief Indica se ocorreu timeout desde a última verificação. */
    bool timeoutOccurred();

  private:
    // TCC: Target CentreCheck
    // MSRC: Minimum Signal Rate Check
    // DSS: Dynamic Spad Selection

    struct SequenceStepEnables
    {
      bool tcc, msrc, dss, pre_range, final_range;
    };

    struct SequenceStepTimeouts
    {
      uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;

      uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
      uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
    };

    I2C * bus;
    uint8_t address;
    uint16_t io_timeout;
    bool did_timeout;
    uint16_t timeout_start_ms;

    uint8_t stop_variable; // read by init and used when starting measurement; is StopVariable field of VL53L0X_DevData_t structure in API
    uint32_t measurement_timing_budget_us;

    bool getSpadInfo(uint8_t * count, bool * type_is_aperture);

    void getSequenceStepEnables(SequenceStepEnables * enables);
    void getSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts);

    bool performSingleRefCalibration(uint8_t vhv_init_byte);

    static uint16_t decodeTimeout(uint16_t value);
    static uint16_t encodeTimeout(uint32_t timeout_mclks);
    static uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
    static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);
};

#endif



