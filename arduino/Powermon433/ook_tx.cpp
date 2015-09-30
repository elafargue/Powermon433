#if defined(DPIN_OOK_TX)
static void TxShortPulse(void)
{
  digitalWrite(DPIN_OOK_TX, HIGH);
  delayMicroseconds(OOK_TX_SHORT);
  digitalWrite(DPIN_OOK_TX, LOW);
}

static void TxRaw(uint8_t v)
{
  for (uint8_t i=0; i<8; ++i)
  {
    TxShortPulse();
    if (v & 0x80)
      delayMicroseconds(OOK_TX_SHORT);
    else
      delayMicroseconds(OOK_TX_LONG);
    v <<= 1;
  }
}

static void TxWireval(uint16_t txId)
{
  // CRC is calculated before txId is added which allows multiple transmitters
  // as the transmitter the receiver isn't tracking will just see the other
  // data is CRC errors
  wireval.crc = crc8(wireval.data.raw, sizeof(wireval.data.raw));
  wireval.data.val16 += txId;

  printWireval();

  // The header is 11111110 and a short duration pause
  // Which will come out as SS SS SS SS SS SS SS SL*1.5
  TxRaw(wireval.hdr);
  delayMicroseconds(OOK_TX_SHORT);
  TxRaw(wireval.data.raw[0]);
  TxRaw(wireval.data.raw[1]);
  TxRaw(wireval.crc);
  // Packet ends with a short pulse otherwise there's no telling what the last
  // bit value was
  TxShortPulse();
}

static void TxIdOnce(uint16_t txId)
{
  // An ID packet is just one where the CRC is good before the ID
  // is subtracted
  wireval.data.val16 = txId;
  TxWireval(0);
}

static void TxInstantOnce(uint16_t txId, uint16_t val)
{
  // last two bits are reserved for packet type
  wireval.data.val16 = (val & 0xfffc) | OOK_PACKET_INSTANT;
  TxWireval(txId);
}

static void TxTempOnce(uint16_t txId, uint8_t temp, uint8_t lowBat)
{
  Serial.print(F("Temperature "));
  wireval.data.raw[0] = (lowBat << 7) | g_TxFlags | OOK_PACKET_TEMP;
  wireval.data.raw[1] = temp;
  TxWireval(txId);
}

static void TxTotalOnce(uint16_t txId, uint16_t val)
{
  Serial.print(F("Total "));
  // last two bits are reserved for packet type
  wireval.data.val16 = (val & 0xfffc) | OOK_PACKET_TOTAL;
  TxWireval(txId);
}

static uint16_t wattsToCnt(uint16_t watts)
{
  return 3600000UL / watts;
}

static uint8_t tempFToCnt(float temp)
{
  return (temp + 28.63) / 0.823;
}
#endif // defined(DPIN_OOK_TX)

static void ookTx(void)
{
#if defined(DPIN_OOK_TX)
  static uint32_t g_LastTx;

  if (digitalReadFast(DPIN_STARTTX_BUTTON) == LOW)
  {
    for (uint8_t i=0; i<4; ++i)
    {
      TxIdOnce(settings.tx_id);
      delay(OOK_ID_DELAY);
    }

    g_LastTx = millis();
  }

  if (g_LastTx != 0 && millis() - g_LastTx > 30500)
  {
    digitalWrite(DPIN_LED, HIGH);
    for (uint8_t i=0; i<3; ++i)
    {
      if (i == 2 || (g_TxCnt >= 0 && g_TxCnt < 4))
        TxInstantOnce(settings.tx_id, wattsToCnt(g_TxWatts));
      else if (g_TxCnt == 4)
        TxTempOnce(settings.tx_id, g_TxTemperature, g_TxLowBat);
      else if (g_TxCnt == 5)
        TxTotalOnce(settings.tx_id, g_TxTotal);
      delay(OOK_TX_DELAY);
    }

    ++g_TxCnt;
    if (g_TxCnt > 5)
      g_TxCnt = 0;

    digitalWrite(DPIN_LED, LOW);
    g_LastTx = millis();
  }
#endif //defined(DPIN_OOK_TX)
}

