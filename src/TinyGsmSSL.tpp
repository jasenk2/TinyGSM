/**
 * @file       TinyGsmSSL.tpp
 * @author     Volodymyr Shymanskyy
 * @license    LGPL-3.0
 * @copyright  Copyright (c) 2016 Volodymyr Shymanskyy
 * @date       Nov 2016
 */

#ifndef SRC_TINYGSMSSL_H_
#define SRC_TINYGSMSSL_H_

#include "TinyGsmCommon.h"

#define TINY_GSM_MODEM_HAS_SSL


template <class modemType>
class TinyGsmSSL {
 public:
  /*
   * SSL functions
   */
  bool addCertificate(const char* filename) {
    return thisModem().addCertificateImpl(filename);
  }
  bool deleteCertificate() {
    return thisModem().deleteCertificateImpl();
  }

  /*
   * CRTP Helper
   */
 protected:
  inline const modemType& thisModem() const {
    return static_cast<const modemType&>(*this);
  }
  inline modemType& thisModem() {
    return static_cast<modemType&>(*this);
  }

  /*
   * Inner Secure Client
   */

 public:
  class GsmClientSecure : public GsmClient {
   public:
    GsmClientSecure() {}

    explicit GsmClientSecure(TinyGsm& modem, uint8_t mux = 0)
        : GsmClient(modem, mux) {}

   public:
    int connect(const char* host, uint16_t port, int timeout_s) overide {
      stop();
      TINY_GSM_YIELD();
      rx.clear();
      sock_connected = at->modemConnect(host, port, mux, true, timeout_s);
      return sock_connected;
    }
  };

  /*
   * SSL functions
   */
 protected:
  bool addCertificateImpl(const char* filename) {
    return thisModem().addCertificateImpl(filename);
  }
  // TINY_GSM_ATTR_NOT_IMPLEMENTED;
  bool deleteCertificateImpl() {
    return thisModem().deleteCertificateImpl();
  }
  //TINY_GSM_ATTR_NOT_IMPLEMENTED;
};

#endif  // SRC_TINYGSMSSL_H_
