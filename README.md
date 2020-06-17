# M480BSP_SPI_FLASH_PDMA
 M480BSP_SPI_FLASH_PDMA

update @ 2020/06/17

1. with SPI inital to access SPI flash , by regular write/read and PDMA write/read

2. platform : M487 EVM with UART terminal (PB.12 , PB.13)

3. SPI flash pin configuration

	- SPI SS : PA.3
	
	- SPI CLK : PA.2	
	
	- SPI MISO : PA.1
	
	- SPI MOSI : PA.0

4. open terminal , press ? , will display description as below 

![image](https://github.com/released/M480BSP_SPI_FLASH_PDMA/blob/master/KEY_questionmark.jpg)
	
5. key 1 ~ 5 , separate function with 1 key

6. key 6 : combo function , erase SPI flash > fill data > write > read > compare

![image](https://github.com/released/M480BSP_SPI_FLASH_PDMA/blob/master/KEY_6.jpg)

7. key 7 : combo function , erase SPI flash > fill data > PDMA write > PDMA read > compare

![image](https://github.com/released/M480BSP_SPI_FLASH_PDMA/blob/master/KEY_7.jpg)
	

