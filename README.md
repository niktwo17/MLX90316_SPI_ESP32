# MLX90316_SPI_ESP32
Library for interfacing MLX90316 SPI with ESP32

- As other librarys using the MLX90316 had problems on the ESP32 I created one, based on a software SPI implementaton, no hardware SPI is needed. Designed for use in a PIO project, no guarantee for Arduino IDE though it should also work.
- Works with "REVISION 011 – AUGUST 17, 2017" from the Melexis datasheet, found here: https://www.melexis.com/en/product/mlx90316/absolute-rotary-position-sensor-ic
- For setting up pins and angle configurations check out MLX.h, in actual main only the radian value is received
