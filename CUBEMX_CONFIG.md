
# Perpheral Configuration in STM32CubeMX

---

### SPI1

- use Motorola format to select "MSB first"

![img/cubemx_spi1.png](img/cubemx_spi1.png)

---

### TIM17

TIM17 must generate something close to 4.9152 MHz

- Prescaler = 0 --> `DIV1`
- Period = (13-1) --> `DIV13`
- Toggle on match --> `DIV2`

128 MHz / 1 / 13 / 2 = **4.9231 MHz**

![img/cubemx_tim17.png](img/cubemx_tim17.png)

---

### Clock for TIM17

![img/cubemx_clock.png](img/cubemx_clock.png)

---

### GPIO / IRQ

![img/cubemx_gpio.png](img/cubemx_gpio.png)



