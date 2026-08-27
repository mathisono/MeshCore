#include "RAK13800EthernetInterface.h"
#include "../../nrf52/EthernetMac.h"
#include <SPI.h>
#include <EthernetUdp.h>

#define PIN_SPI1_MISO (29)
#define PIN_SPI1_MOSI (30)
#define PIN_SPI1_SCK  (3)

SPIClass ETHERNET_SPI_PORT(NRF_SPIM1, PIN_SPI1_MISO, PIN_SPI1_SCK, PIN_SPI1_MOSI);

#define PIN_ETHERNET_POWER_EN WB_IO2
#define PIN_ETHERNET_RESET 21
#define PIN_ETHERNET_SS 26

#ifdef WITH_W5100S_POE
  // The RAK19018/Silvertel converter needs time with the W5100S active load
  // before the Ethernet library performs its disruptive PHY/DHCP bring-up.
  #ifndef ETH_POE_DEFER_MS
    #define ETH_POE_DEFER_MS 6000UL
  #endif
  #ifndef ETH_POE_RETRY_MS
    #define ETH_POE_RETRY_MS 30000UL
  #endif
  #ifndef ETH_POE_FALLBACK_IP
    #define ETH_POE_FALLBACK_IP 192,168,1,50
  #endif
  #ifndef ETH_POE_FALLBACK_GATEWAY
    #define ETH_POE_FALLBACK_GATEWAY 192,168,1,1
  #endif
  #ifndef ETH_POE_FALLBACK_SUBNET
    #define ETH_POE_FALLBACK_SUBNET 255,255,255,0
  #endif
#endif

static void eth_init_spi_and_pins() {
  // WB_IO2 is asserted by the early RAK4631 constructor. Never pulse RESET
  // here: dropping W5100S PHY/link can collapse PoE power on the RAK19018.
  pinMode(PIN_ETHERNET_RESET, OUTPUT);
  digitalWrite(PIN_ETHERNET_RESET, HIGH);

  ETHERNET_SPI_PORT.begin();
  Ethernet.init(ETHERNET_SPI_PORT, PIN_ETHERNET_SS);
}

static bool eth_bring_up(uint8_t mac[6]) {
#ifdef WITH_W5100S_POE
  ETHERNET_DEBUG_PRINTLN("Trying DHCP (PoE-safe deferred bring-up)...");
  if (Ethernet.begin(mac, 12000, 4000) == 0) {
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      ETHERNET_DEBUG_PRINTLN("RAK13800 hardware not found; will retry later");
      return false;
    }

    ETHERNET_DEBUG_PRINTLN("DHCP failed; using PoE fallback address");
    IPAddress ip(ETH_POE_FALLBACK_IP);
    IPAddress gateway(ETH_POE_FALLBACK_GATEWAY);
    IPAddress subnet(ETH_POE_FALLBACK_SUBNET);
    Ethernet.begin(mac, ip, gateway, gateway, subnet);
  }
#elif defined(ETHERNET_STATIC_IP) && defined(ETHERNET_STATIC_GATEWAY) && defined(ETHERNET_STATIC_SUBNET) && defined(ETHERNET_STATIC_DNS)
  IPAddress ip(ETHERNET_STATIC_IP);
  IPAddress gateway(ETHERNET_STATIC_GATEWAY);
  IPAddress subnet(ETHERNET_STATIC_SUBNET);
  IPAddress dns(ETHERNET_STATIC_DNS);
  Ethernet.begin(mac, ip, dns, gateway, subnet);
#else
  if (Ethernet.begin(mac) == 0) {
    ETHERNET_DEBUG_PRINTLN("Failed to initialize RAK13800 hardware.");
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      ETHERNET_DEBUG_PRINTLN("Ethernet hardware not found.");
    } else if (Ethernet.linkStatus() == LinkOFF) {
      ETHERNET_DEBUG_PRINTLN("Ethernet cable not connected.");
    } else {
      ETHERNET_DEBUG_PRINTLN("DHCP failed for unknown reason.");
    }
    return false;
  }
#endif

  ETHERNET_DEBUG_PRINTLN("Ethernet begin complete");
  ETHERNET_DEBUG_PRINT_IP("IP Address", Ethernet.localIP());
  ETHERNET_DEBUG_PRINT_IP("Subnet Mask", Ethernet.subnetMask());
  ETHERNET_DEBUG_PRINT_IP("Gateway", Ethernet.gatewayIP());
  ETHERNET_DEBUG_PRINT_IP("DNS", Ethernet.dnsServerIP());
  return true;
}

void RAK13800EthernetInterface::resetRxState(uint8_t slot) {
  rx_state[slot].state = RX_WAIT_START;
  rx_state[slot].frame_len = 0;
  rx_state[slot].rx_len = 0;
}

void RAK13800EthernetInterface::clearBuffers() {
  send_queue_len = 0;
  _response_owner = -1;
  _stream_response = false;
  _release_after_flush = false;
  _response_activity = 0;
  _rr = 0;
  for (uint8_t i = 0; i < MAX_ETH_CLIENTS; i++) {
    resetRxState(i);
  }
}

bool RAK13800EthernetInterface::begin() {
  uint8_t mac[6];
  generateEthernetMac(mac);
  ETHERNET_DEBUG_PRINTLN(
      "Ethernet MAC: %02X:%02X:%02X:%02X:%02X:%02X",
      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

#ifdef WITH_W5100S_POE
  // W5100S load activation happens in RAK4631Board::begin(). Delay the
  // Ethernet library reset/DHCP stage until the PoE converter is latched.
  _startedAt = millis();
  _lastBringUpAttempt = 0;
  ETHERNET_DEBUG_PRINTLN("Ethernet bring-up deferred for PoE stability");
  return true;
#else
  eth_init_spi_and_pins();
  if (!eth_bring_up(mac)) return false;
  server.begin();
  ETHERNET_DEBUG_PRINTLN("listening on TCP port: %d", ETHERNET_TCP_PORT);
  return true;
#endif
}

#ifdef WITH_W5100S_POE
bool RAK13800EthernetInterface::bringUpHardware() {
  eth_init_spi_and_pins();
  uint8_t mac[6];
  generateEthernetMac(mac);
  if (!eth_bring_up(mac)) return false;
  server.begin();
  ETHERNET_DEBUG_PRINTLN("listening on TCP port: %d", ETHERNET_TCP_PORT);
  return true;
}
#endif

void RAK13800EthernetInterface::enable() {
  if (_isEnabled) return;
  _isEnabled = true;
  clearBuffers();
}

void RAK13800EthernetInterface::disable() {
  _isEnabled = false;
  for (uint8_t i = 0; i < MAX_ETH_CLIENTS; i++) {
    clients[i].stop();
    resetRxState(i);
  }
  _connected = false;
  clearBuffers();
}

void RAK13800EthernetInterface::serviceConnections() {
  EthernetClient newClient = server.accept();
  if (newClient) {
    int slot = -1;
    for (uint8_t i = 0; i < MAX_ETH_CLIENTS; i++) {
      if (!clients[i].connected()) {
        slot = i;
        break;
      }
    }

    if (slot >= 0) {
      clients[slot].stop();
      clients[slot] = newClient;
      resetRxState((uint8_t)slot);
      IPAddress remoteIp = clients[slot].remoteIP();
      ETHERNET_DEBUG_PRINTLN(
          "New client accepted in slot %d: %u.%u.%u.%u:%u",
          slot, remoteIp[0], remoteIp[1], remoteIp[2], remoteIp[3],
          clients[slot].remotePort());
    } else {
      ETHERNET_DEBUG_PRINTLN("Rejecting client: all %d slots are busy", MAX_ETH_CLIENTS);
      newClient.stop();
    }
  }

  bool any = false;
  for (uint8_t i = 0; i < MAX_ETH_CLIENTS; i++) {
    if (clients[i].connected()) {
      any = true;
      continue;
    }

    // If the response owner disappears during a long response, keep the
    // stream lock until its END marker is generated, but discard those frames.
    // This prevents a partial response from leaking to a different client.
    if (_response_owner == (int8_t)i) {
      _response_owner = -1;
      if (!_stream_response) {
        _release_after_flush = false;
        _response_activity = 0;
      }
    }

    clients[i].stop();
    resetRxState(i);
  }
  _connected = any;
}

void RAK13800EthernetInterface::expireResponseAffinity() {
  if ((_response_owner < 0 && !_stream_response) || _response_activity == 0) return;

  if ((unsigned long)(millis() - _response_activity) > ETH_RESPONSE_AFFINITY_TIMEOUT_MS) {
    ETHERNET_DEBUG_PRINTLN("Response affinity timed out; releasing command owner");
    _response_owner = -1;
    _stream_response = false;
    _release_after_flush = false;
    _response_activity = 0;
  }
}

void RAK13800EthernetInterface::flushOutbound() {
  while (send_queue_len > 0) {
    Frame &frame = send_queue[0];
    uint8_t packet[3 + MAX_FRAME_SIZE];
    packet[0] = '>';
    packet[1] = frame.len & 0xFF;
    packet[2] = frame.len >> 8;
    memcpy(&packet[3], frame.buf, frame.len);

    if (frame.target < 0) {
      for (uint8_t i = 0; i < MAX_ETH_CLIENTS; i++) {
        if (clients[i].connected()) {
          clients[i].write(packet, 3 + frame.len);
        }
      }
    } else if (frame.target < MAX_ETH_CLIENTS && clients[frame.target].connected()) {
      clients[frame.target].write(packet, 3 + frame.len);
    }

    send_queue_len--;
    for (uint8_t i = 0; i < send_queue_len; i++) {
      send_queue[i] = send_queue[i + 1];
    }
  }

  // Ordinary responses release after their queued frames have actually been
  // written. GET_CONTACTS releases only after RESP_END_OF_CONTACTS is flushed.
  if (_release_after_flush) {
    _response_owner = -1;
    _stream_response = false;
    _release_after_flush = false;
    _response_activity = 0;
  }
}

size_t RAK13800EthernetInterface::readFrameFromClient(uint8_t slot, uint8_t dest[]) {
  EthernetClient &client = clients[slot];
  RxState &rx = rx_state[slot];

  while (client.connected() && client.available()) {
    int value = client.read();
    if (value < 0) break;
    uint8_t byte = (uint8_t)value;

    switch (rx.state) {
      case RX_WAIT_START:
        if (byte == '<') rx.state = RX_WAIT_LEN_LO;
        break;

      case RX_WAIT_LEN_LO:
        rx.frame_len = byte;
        rx.state = RX_WAIT_LEN_HI;
        break;

      case RX_WAIT_LEN_HI:
        rx.frame_len |= ((uint16_t)byte) << 8;
        rx.rx_len = 0;
        if (rx.frame_len == 0) {
          resetRxState(slot);
        } else if (rx.frame_len > MAX_FRAME_SIZE) {
          ETHERNET_DEBUG_PRINTLN("RX[%d] oversized frame (%u); closing client", slot, rx.frame_len);
          client.stop();
          resetRxState(slot);
          return 0;
        } else {
          rx.state = RX_WAIT_PAYLOAD;
        }
        break;

      case RX_WAIT_PAYLOAD:
        rx.buf[rx.rx_len++] = byte;
        if (rx.rx_len >= rx.frame_len) {
          size_t len = rx.frame_len;
          memcpy(dest, rx.buf, len);
          resetRxState(slot);
          return len;
        }
        break;
    }
  }

  return 0;
}

size_t RAK13800EthernetInterface::writeFrame(const uint8_t src[], size_t len) {
  if (!_isEnabled || len == 0 || len > MAX_FRAME_SIZE) return 0;
#ifdef WITH_W5100S_POE
  if (!_hwReady) return 0;
#endif

  bool isPush = src[0] >= COMPANION_PUSH_MIN;
  int8_t target = -1;

  if (isPush) {
    if (!_connected) return 0;
    target = -1; // unsolicited events are visible to all connected apps
  } else if (_response_owner >= 0) {
    target = _response_owner;
  } else if (_stream_response) {
    // The stream owner disconnected. Consume the remaining generated response
    // without sending it anywhere; release only when the stream END arrives.
    _response_activity = millis();
    if (src[0] == COMPANION_RESP_END_OF_CONTACTS) {
      _stream_response = false;
      _release_after_flush = false;
      _response_activity = 0;
    }
    return len;
  } else {
    return 0;
  }

  if (send_queue_len >= ETH_FRAME_QUEUE_SIZE) {
    ETHERNET_DEBUG_PRINTLN("writeFrame(): queue full, dropping code=0x%02X", src[0]);
    return 0;
  }

  send_queue[send_queue_len].target = target;
  send_queue[send_queue_len].len = (uint8_t)len;
  memcpy(send_queue[send_queue_len].buf, src, len);
  send_queue_len++;

  if (!isPush) {
    _response_activity = millis();
    if (!_stream_response || src[0] == COMPANION_RESP_END_OF_CONTACTS) {
      _release_after_flush = true;
    }
  }

  return len;
}

size_t RAK13800EthernetInterface::checkRecvFrame(uint8_t dest[]) {
  if (!_isEnabled) return 0;
#ifdef WITH_W5100S_POE
  if (!_hwReady) return 0;
#endif

  serviceConnections();
  expireResponseAffinity();
  flushOutbound();

  // Companion lacks transaction IDs, so process at most one command-response
  // transaction at a time. Other clients remain connected and can continue to
  // receive broadcast push events while their inbound command bytes wait.
  if (_response_owner >= 0 || _stream_response) return 0;

  for (uint8_t n = 0; n < MAX_ETH_CLIENTS; n++) {
    uint8_t slot = (_rr + n) % MAX_ETH_CLIENTS;
    if (!clients[slot].connected()) continue;

    size_t len = readFrameFromClient(slot, dest);
    if (len > 0) {
      _response_owner = (int8_t)slot;
      _stream_response = dest[0] == COMPANION_CMD_GET_CONTACTS;
      _release_after_flush = false;
      _response_activity = millis();
      _rr = (slot + 1) % MAX_ETH_CLIENTS;
      ETHERNET_DEBUG_PRINTLN("RX[%d] command=0x%02X len=%u", slot, dest[0], (unsigned)len);
      return len;
    }
  }

  return 0;
}

void RAK13800EthernetInterface::loop() {
#ifdef WITH_W5100S_POE
  if (!_hwReady) {
    unsigned long now = millis();
    if ((unsigned long)(now - _startedAt) < ETH_POE_DEFER_MS) return;
    if (_lastBringUpAttempt != 0 &&
        (unsigned long)(now - _lastBringUpAttempt) < ETH_POE_RETRY_MS) return;

    _lastBringUpAttempt = now;
    _hwReady = bringUpHardware();
    return;
  }
#endif

  Ethernet.maintain();
}
