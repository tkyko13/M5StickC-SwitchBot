
void i2sInit() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
    .sample_rate =  SAMPLING_FREQUENCY,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // is fixed at 12bit, stereo, MSB
    .channel_format = I2S_CHANNEL_FMT_ALL_RIGHT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 2,
    .dma_buf_len = 128,
  };

  i2s_pin_config_t pin_config;
  pin_config.bck_io_num   = I2S_PIN_NO_CHANGE;
  pin_config.ws_io_num    = PIN_CLK;
  pin_config.data_out_num = I2S_PIN_NO_CHANGE;
  pin_config.data_in_num  = PIN_DATA;


  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  i2s_set_clk(I2S_NUM_0, SAMPLING_FREQUENCY, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
}

void mic_fft_task (void* arg) {
  while (1) {
    i2s_read_bytes(I2S_NUM_0, (char*)BUFFER, READ_LEN, (100 / portTICK_RATE_MS));
    adcBuffer = (uint16_t *)BUFFER;

    fft();

    vTaskDelay(500 / portTICK_RATE_MS);
  }
}

void fft() {
  for (int i = 0; i < FFTsamples; i++) {
    unsigned long t = micros();
    vReal[i] = adcBuffer[i];
    vImag[i] = 0;
    while ((micros() - t) < sampling_period_us) ;
  }

  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);  // 窓関数
  FFT.Compute(FFT_FORWARD); // FFT処理(複素数で計算)
  FFT.ComplexToMagnitude(); // 複素数を実数に変換

  int nsamples = FFTsamples / 2;

  if (TH_FREQ_BAND == -1) {
    float avg = 0;
    for (int band = 5; band < nsamples; band++) {
      avg += vReal[band] / dmax;
    }
    avg = avg / (nsamples - 5);
    //    Serial.print("volum avg : ");
    //    Serial.println(avg);
    if (avg > TH_VOLUME) {
      sendFlg = true;
      //      doScan = true;
    }
  }
  else {
    float d = vReal[TH_FREQ_BAND] / dmax;
    if (d > TH_VOLUME) {
      sendFlg = true;
      //      doScan = true;
    }
  }
}
