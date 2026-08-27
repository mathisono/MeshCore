#pragma once

#include "../SerialEthernetInterface.h"
#include <SPI.h>
#include <RAK13800_W5100S.h>

// The W5100S has four hardware sockets. Keep one for the listening socket and
// allow three simultaneous MeshCore Companion clients by default.
#ifndef MAX_ETH_CLIENTS
  #define MAX_ETH_CLIENTS 3
#endif

#ifndef ETH_FRAME_QUEUE_SIZE
  #define ETH_FRAME_QUEUE_SIZE 8
#endif

// The Companion protocol has no request/transaction identifier. Keep response
// ownership pinned to the client that issued a command until that response has
// been flushed. This also prevents long responses (notably GET_CONTACTS) from
// being redirected if another connected client sends a command mid-stream.
#ifndef ETH_RESPONSE_AFFINITY_TIMEOUT_MS
  #define ETH_RESPONSE_AFFINITY_TIMEOUT_MS 30000UL
#endif

class RAK13800EthernetInterface : public SerialEthernetInterface {
  enum RxParseState : uint8_t {
    RX_WAIT_START = 0,
    RX_WAIT_LEN_LO,
    RX_WAIT_LEN_HI,
    RX_WAIT_PAYLOAD
  };

  enum : uint8_t {
    COMPANION_CMD_GET_CONTACTS = 4,
    COMPANION_RESP_END_OF_CONTACTS = 4,
    COMPANION_PUSH_MIN = 0x80
  };

  struct RxState {
    RxParseState state;
    uint16_t frame_len;
    uint16_t rx_len;
    uint8_t buf[MAX_FRAME_SIZE];
  };

  struct Frame {
    int8_t target;   // -1 = broadcast, otherwise client slot
    uint8_t len;
    uint8_t buf[MAX_FRAME_SIZE];
  };

  bool _isEnabled;
  bool _connected;
  EthernetServer server;
  EthernetClient clients[MAX_ETH_CLIENTS];
  RxState rx_state[MAX_ETH_CLIENTS];

  Frame send_queue[ETH_FRAME_QUEUE_SIZE];
  uint8_t send_queue_len;

  int8_t _response_owner;
  bool _stream_response;
  bool _release_after_flush;
  unsigned long _response_activity;
  uint8_t _rr;

#ifdef WITH_W5100S_POE
  bool _hwReady;
  unsigned long _startedAt;
  unsigned long _lastBringUpAttempt;
  bool bringUpHardware();
#endif

  void resetRxState(uint8_t slot);
  void clearBuffers();
  void serviceConnections();
  void flushOutbound();
  void expireResponseAffinity();
  size_t readFrameFromClient(uint8_t slot, uint8_t dest[]);

public:
  RAK13800EthernetInterface() : server(EthernetServer(ETHERNET_TCP_PORT)) {
    _isEnabled = false;
    _connected = false;
#ifdef WITH_W5100S_POE
    _hwReady = false;
    _startedAt = 0;
    _lastBringUpAttempt = 0;
#endif
    clearBuffers();
  }

  bool begin();
  void loop() override;

  void enable() override;
  void disable() override;
  bool isEnabled() const override { return _isEnabled; }
  bool isConnected() const override { return _connected; }
  bool isWriteBusy() const override { return send_queue_len > 0; }

  size_t writeFrame(const uint8_t src[], size_t len) override;
  size_t checkRecvFrame(uint8_t dest[]) override;

  // Framing and routing are fully overridden above. These are retained only to
  // satisfy SerialEthernetInterface's low-level pure-virtual contract.
  int available() override { return 0; }
  int read() override { return -1; }
  size_t write(const uint8_t *buf, size_t size) override {
    (void)buf;
    (void)size;
    return 0;
  }
};
