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
  bool addCACert(const char* ca){
    return thisModem().addCACertImpl(ca);
  }
  bool addCertificate(const char* filename) {    
    return thisModem().addCertificateImpl(filename);
  }
  bool deleteCertificate() {
    return thisModem().deleteCertificateImpl();
  }
  bool addKey(const char* filename) {
    return thisModem().addKeyImpl(filename);
  }
  bool deleteKey() {
    return thisModem().deleteKeyImpl();
  }
  bool hasSSL()
  {
    return true;
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
   * SSL functions
   */
 protected:
  bool addCACertImpl(const char* ca){
    return thisModem().addCACertImpl(ca);
  }
    bool addCertificateImpl(const char* filename) {
    return thisModem().addCertificateImpl(filename);
  }
  bool addKeyImpl(const char* filename) {
    return thisModem().addKeyImpl(filename);
  }  // TINY_GSM_ATTR_NOT_IMPLEMENTED;
  bool deleteCertificateImpl() {
    return thisModem().deleteCertificateImpl();
  }
  bool deleteKeyImpl() {
    return thisModem().deleteKeyImpl();
  }
  //TINY_GSM_ATTR_NOT_IMPLEMENTED;
};

#endif  // SRC_TINYGSMSSL_H_
