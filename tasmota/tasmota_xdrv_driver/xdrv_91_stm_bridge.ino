/*
  xdrv_91_stm_bridge.ino - TCP to STM32 bootloader bridge

  Copyright (C) 2024  Theo Arends, Stephan Hadinger, and Jeroen van Straten

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef USE_STM_BRIDGE

#define XDRV_91                    91

#ifndef STM_BRIDGE_BUF_SIZE
#define STM_BRIDGE_BUF_SIZE    255  // size of the buffer, above 132 required for efficient XMODEM
#endif

WiFiServer   *server_stm_boot = nullptr;
WiFiClient   client_stm_boot;
WiFiServer   *server_stm_vcp = nullptr;
WiFiClient   client_stm_vcp;
uint8_t      *stm_buf = nullptr;     // data transfer buffer
bool         stm_ip_filter_enabled = false;
IPAddress    stm_ip_filter;
bool         stm_serial = false;
bool         stm_in_bootloader = false;

#include <TasmotaSerial.h>
TasmotaSerial *STMSerial = nullptr;

const char kSTMCommands[] PROGMEM = "STM" "|"    // prefix
  "Start" "|" "Baudrate" "|" "Config"
  ;

void (* const STMCommand[])(void) PROGMEM = {
  &CmndSTMStart, &CmndSTMBaudrate, &CmndSTMConfig
  };


// Reconfigures the serial port
void STMSerialReconfig(void) {
  if (stm_in_bootloader) {
    if (!STMSerial->begin(115200, ConvertSerialConfig(ParseSerialConfig("8E1")))) {
      AddLog(LOG_LEVEL_DEBUG, PSTR(D_LOG_STM "failed reinit serial"));
    }
  } else {
    if (!STMSerial->begin(Settings->stm_baudrate * 1200, ConvertSerialConfig(0x7F & Settings->stm_config))) {
      AddLog(LOG_LEVEL_DEBUG, PSTR(D_LOG_STM "failed reinit serial"));
    }
  }
}

//
// Resets STM32 to bootloader or to user code
//
void STMReset(bool bootloader) {
  if (bootloader) {
    AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_STM "Resetting into bootloader"));
    digitalWrite(Pin(GPIO_STM_BOOT0), HIGH);
  } else {
    AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_STM "Resetting to user code"));
    digitalWrite(Pin(GPIO_STM_BOOT0), LOW);
  }
  digitalWrite(Pin(GPIO_STM_NRST), LOW);
  stm_in_bootloader = bootloader;
  STMSerialReconfig();
  if (client_stm_vcp) {
    if (bootloader) {
      client_stm_vcp.print("\r\nReset to bootloader...\r\n");
    } else {
      client_stm_vcp.print("\r\nReset to application...\r\n");
    }
  }
  digitalWrite(Pin(GPIO_STM_NRST), HIGH);
}

//
// Called at event loop
//
void STMLoop(void)
{
  uint8_t c;
  bool busy;    // did we transfer some data?
  int32_t buf_len;

  // check for a new bootloader client connection
  if ((server_stm_boot) && (server_stm_boot->hasClient())) {
    WiFiClient new_client = server_stm_boot->available();

    AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_STM "Got bootloader connection from %s"), new_client.remoteIP().toString().c_str());
    // Check for IP filtering if it's enabled.
    if (stm_ip_filter_enabled) {
      if (stm_ip_filter != new_client.remoteIP()) {
        AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_STM "Rejected due to filtering"));
        new_client.stop();
      } else {
        AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_STM "Allowed through filter"));
      }
    }

    if (client_stm_boot) {
      AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_STM "Kicking previous connection"));
      client_stm_boot.stop();
    }

    // Reset into bootloader automatically when something connects to the
    // bootloader channel.
    STMReset(true);
    
    client_stm_boot = new_client;
  }

  // Handle connection close from bootloader channel to enter user code.
  if (stm_in_bootloader && !client_stm_boot) {
    STMReset(false);
  }

  // check for a new virtual com port client connection
  if ((server_stm_vcp) && (server_stm_vcp->hasClient())) {
    WiFiClient new_client = server_stm_vcp->available();

    AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_STM "Got VCP connection from %s"), new_client.remoteIP().toString().c_str());
    // Check for IP filtering if it's enabled.
    if (stm_ip_filter_enabled) {
      if (stm_ip_filter != new_client.remoteIP()) {
        AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_STM "Rejected due to filtering"));
        new_client.stop();
      } else {
        AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_STM "Allowed through filter"));
      }
    }

    if (client_stm_vcp) {
      AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_STM "Kicking previous connection"));
      client_stm_vcp.stop();
    }
    
    client_stm_vcp = new_client;
  }

  WiFiClient &client = stm_in_bootloader ? client_stm_boot : client_stm_vcp;
  
  do {
    busy = false;       // exit loop if no data was transferred

    // start reading the UART, this buffer can quickly overflow
    buf_len = 0;
    while ((buf_len < STM_BRIDGE_BUF_SIZE) && (STMSerial->available())) {
      c = STMSerial->read();
      if (c >= 0) {
        stm_buf[buf_len++] = c;
        busy = true;
      }
    }
    if (buf_len > 0) {
      AddLog(LOG_LEVEL_DEBUG_MORE, PSTR(D_LOG_STM "from STM: %*_H"), buf_len, stm_buf);

      if (client) { client.write(stm_buf, buf_len); }
    }

    // handle data received from TCP
    buf_len = 0;
    while (client && (buf_len < STM_BRIDGE_BUF_SIZE) && (client.available())) {
      c = client.read();
      if (c >= 0) {
        stm_buf[buf_len++] = c;
        busy = true;
      }
    }
    if (buf_len > 0) {
      AddLog(LOG_LEVEL_DEBUG_MORE, PSTR(D_LOG_STM "to STM (boot=%d): %*_H"), stm_in_bootloader, buf_len, stm_buf);
      STMSerial->write(stm_buf, buf_len);
    }

    yield();    // avoid WDT if heavy traffic
  } while (busy);

  // Send incoming virtual COM port data to /dev/null when in bootloader
  while (stm_in_bootloader && client_stm_vcp && client_stm_vcp.available()) {
    client.read();
  }
}

/********************************************************************************************/

void STMInit(void) {
  if (PinUsed(GPIO_STM_RX) && PinUsed(GPIO_STM_TX) && PinUsed(GPIO_STM_BOOT0) && PinUsed(GPIO_STM_NRST)) {
    pinMode(Pin(GPIO_STM_BOOT0), OUTPUT);
    digitalWrite(Pin(GPIO_STM_BOOT0), HIGH);
    pinMode(Pin(GPIO_STM_NRST), OUTPUT);
    digitalWrite(Pin(GPIO_STM_NRST), HIGH);

    if (0 == (0x80 & Settings->stm_config)) {  // !0x80 means unitialized
      Settings->stm_config = 0x80 | ParseSerialConfig("8N1");  // default as 8N1 for backward compatibility
    }
    stm_buf = (uint8_t*) malloc(STM_BRIDGE_BUF_SIZE);
    if (!stm_buf) { 
      AddLog(LOG_LEVEL_ERROR, PSTR(D_LOG_STM "could not allocate buffer"));
      return;
    }

    if (!Settings->stm_baudrate) { 
      Settings->stm_baudrate = 115200 / 1200;
    }

    STMSerial = new TasmotaSerial(Pin(GPIO_STM_RX), Pin(GPIO_STM_TX), TasmotaGlobal.seriallog_level ? 1 : 2, 0, STM_BRIDGE_BUF_SIZE);   // set a receive buffer of 256 bytes
    stm_serial = STMSerial->begin(Settings->stm_baudrate * 1200, ConvertSerialConfig(0x7F & Settings->stm_config));
    if (stm_serial) {
      if (STMSerial->hardwareSerial()) {
        ClaimSerial();
      }
#ifdef ESP32
      AddLog(LOG_LEVEL_DEBUG, PSTR(D_LOG_STM "Serial UART%d"), STMSerial->getUart());
#endif

      STMReset(false);
    } else {
      AddLog(LOG_LEVEL_DEBUG, PSTR(D_LOG_STM "failed init serial"));
    }
  }
}

/*********************************************************************************************\
 * Commands
\*********************************************************************************************/

//
// Command `STMStart`
// Params: port,<IPv4 allow>
//
void CmndSTMStart(void) {
  int32_t tcp_port = XdrvMailbox.payload;
  if (ArgC() == 2) {
    char sub_string[XdrvMailbox.data_len];
    stm_ip_filter.fromString(ArgV(sub_string, 2));
    stm_ip_filter_enabled = true;
  } else {
    // Disable whitelist if previously set
    stm_ip_filter_enabled = false;
  }

  if (server_stm_boot) {
    AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_STM "Stopping bootloader TCP server"));

    server_stm_boot->stop();
    delete server_stm_boot;
    server_stm_boot = nullptr;

    client_stm_boot.stop();
  }
  if (server_stm_vcp) {
    AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_STM "Stopping virtual COM port TCP server"));

    server_stm_vcp->stop();
    delete server_stm_vcp;
    server_stm_vcp = nullptr;

    client_stm_vcp.stop();
  }
  if (tcp_port > 0) {
    AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_STM "Starting TCP server on port %d (bootloader) and %d (virtual COM port)"), tcp_port, tcp_port + 1);
    if (stm_ip_filter_enabled) {
      AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_STM "Filtering %s"), stm_ip_filter.toString().c_str());
    }
    server_stm_boot = new WiFiServer(tcp_port);
    server_stm_boot->begin(); // start TCP server
    server_stm_boot->setNoDelay(true);
    server_stm_vcp = new WiFiServer(tcp_port + 1);
    server_stm_vcp->begin(); // start TCP server
    server_stm_vcp->setNoDelay(true);
  }

  ResponseCmndDone();
}

void CmndSTMBaudrate(void) {
  if ((XdrvMailbox.payload >= 1200) && (XdrvMailbox.payload <= 115200)) {
    XdrvMailbox.payload /= 1200;  // Make it a valid baudrate
    if (Settings->stm_baudrate != XdrvMailbox.payload) {
      Settings->stm_baudrate = XdrvMailbox.payload;
      STMSerialReconfig();
    }
  }
  ResponseCmndNumber(Settings->stm_baudrate * 1200);
}

void CmndSTMConfig(void) {
  if (XdrvMailbox.data_len > 0) {
    uint8_t serial_config = ParseSerialConfig(XdrvMailbox.data);
    if ((serial_config >= 0) && (Settings->stm_config != (0x80 | serial_config))) {
      Settings->stm_config = 0x80 | serial_config; // default 0x00 should be 8N1
    }
  }
  ResponseCmndChar_P(GetSerialConfig(0x7F & Settings->stm_config).c_str());
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xdrv91(uint32_t function)
{
  bool result = false;

  if (FUNC_PRE_INIT == function) {
    STMInit();
  } else if (stm_serial) {
    switch (function) {
      case FUNC_LOOP:
        STMLoop();
        break;
      case FUNC_COMMAND:
        result = DecodeCommand(kSTMCommands, STMCommand);
        break;
      case FUNC_ACTIVE:
        result = true;
        break;
    }
  }
  return result;
}

#endif // USE_STM_BRIDGE
