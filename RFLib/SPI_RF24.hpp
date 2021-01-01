/*
 * SPI.hpp
 *
 *  Created on: 29 dic 2020
 *      Author: ccattaneo
 */

#ifndef SPI_RF24_HPP_
#define SPI_RF24_HPP_
#include "RF24_config.hpp"

class SPI_RF24 {

public:

    /**
    * SPI constructor
    */
	SPI_RF24();

    /**
    * Start SPI
    */
    void begin(uint32_t spi_speed = RF24_SPI_SPEED);

    /**
    * Transfer a single byte
    * @param tx Byte to send
    * @return Data returned via spi
    */
    uint8_t transfer(uint8_t tx);

    /**
    * Transfer a buffer of data
    * @param tbuf Transmit buffer
    * @param rbuf Receive buffer
    * @param len Length of the data
    */
    void transfernb(char* tbuf, char* rbuf, uint32_t len);

    /**
    * Transfer a buffer of data without an rx buffer
    * @param buf Pointer to a buffer of data
    * @param len Length of the data
    */
    void transfern(char* buf, uint32_t len)
    {
        transfernb(buf, buf, len);
    }

    ~SPI_RF24();

private:
    SPI_HandleTypeDef hspi1;
    int fd;
    uint32_t _spi_speed;
    bool spiIsInitialized = false;
    void init(uint32_t spi_speed = RF24_SPI_SPEED);
    void MX_SPI1_Init(void);
};

#endif /* SPI_RF24_HPP_ */
