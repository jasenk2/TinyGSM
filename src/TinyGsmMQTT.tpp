/**
 * @file       TinyGsmMQTT.tpp
 * @author     Jasen Kolev
 * @license    LGPL-3.0
 * @copyright  Copyright (c) 2020 Jasen Kolev
 * @date       Nov 2020
 */


#ifndef SRC_TINYGSMMQTT_H_
#define SRC_TINYGSMMQTT_H_

#include "TinyGsmCommon.h"

#define TINY_GSM_MODEM_HAS_MQTT

template <class modemType>
class TinyGsmMQTT {
  /*
   * MQTT functions
   */
 public:
  bool mqttConnect(const char* url, uint16_t port, 
                   bool ssl=false,uint8_t QoS=0, const char* user="", const char* pwd="",bool retain=false) {
    isConnected = thisModem().mqttConnectImpl(url, port, retain, QoS, user, pwd, ssl);
    return isConnected;
  }
  bool mqttDisconnect() {
    return thisModem().mqttDisconnectImpl();
  }
  bool mqttConnected(){
    return isConnected;  
  }
  bool mqttSub(const char* topic){
    return thisModem().mqttSubImpl(topic);
  } 
  bool mqttUnSub(const char* topic){
    return thisModem().mqttUnSubImpl(topic);
  } 
  String mqttRecv(const char* topic){
    return thisModem().mqttRecvImpl(topic);
  }
  bool mqttPub(const char* topic, String msg, uint8_t QoS=0,bool retain=false){
    return thisModem().mqttPubImpl(topic,msg,QoS,retain);
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
   * WiFi functions
   */

  bool mqttConnectImpl(const char* url, uint16_t port, 
                        bool retain=false,uint8_t QoS=0, const char* user="", const char* pwd="",bool ssl=false)
  
  {
    return thisModem().mqttConnectImpl(url, port, retain, QoS, user, pwd, ssl);
  }
                        
  bool mqttDisconnectImpl() {
      return thisModem.mqttDisconnectImpl();
  }
  // vars
  bool isConnected;

};

#endif  // SRC_TINYGSMMQTT








