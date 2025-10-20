**TALLER**

Maria Alejandra Bernal Salazar 
Codigó: 55622019

1. Comparativa entre lenguaje de programación y lenguaje de descripción de hardware.

2. Comparativa entre microcontrolador, microprocesador FPGA, PLD.

3. Que lenguaje son mas usados en prog. de hardware.

4. Proponga una solución tecnologica embebida mediante:
-Proceso microcontrolado. ej:esp32
-Programacion de hardware. ej: basys

5. Desarrolle el codigo y documente los hallazgos que ventajas y desventajas tiene cada sistema.

<center>Solución</center>

**1.** 
| **Criterio** | **Lenguaje de Programación** | **Lenguaje de Descripción de Hardware (HDL)** |
|---------------|-------------------------------|-----------------------------------------------|
| **Propósito principal** | Crear programas que se ejecutan secuencialmente en un procesador (software). | Describir y modelar el comportamiento y estructura de circuitos digitales (hardware). |
| **Ejemplos comunes** | C, C++, Python, Java | VHDL, Verilog, SystemVerilog |
| **Ejecución** | Se compila o interpreta para ejecutarse en una CPU. | Se sintetiza para implementarse físicamente en FPGA o ASIC. |
| **Forma de procesamiento** | Secuencial: las instrucciones se ejecutan una después de otra. | Paralela: las operaciones ocurren simultáneamente según las señales y el reloj. |
| **Unidad básica** | Instrucción o función. | Componente o módulo (flip-flop, compuerta, registro, etc.). |
| **Tiempo y sincronización** | Controlado por el flujo del programa. | Controlado por señales, relojes y eventos en el tiempo. |
| **Salida del diseño** | Programa ejecutable. | Descripción de circuito sintetizable o simulable. |
| **Nivel de abstracción** | Alto nivel (enfocado en lógica de control o datos). | Bajo o medio nivel (enfocado en conexiones físicas y comportamiento de señales). |
| **Uso típico en ingeniería** | Desarrollo de firmware o software de control. | Diseño y simulación de sistemas digitales, FPGAs o microprocesadores. |

Ejemplo comparativo entre ambos lenguajes 


**Lenguaje de programación (C):**
```c
int suma(int a, int b) {
    return a + b;
}
```
 E este código se observa que se llama a una función denominada (suma), la cual recibe dos números, (a y b), y devuelve el resultado de su suma. El programa se ejecuta de forma secuencial, es decir, paso a paso, procesando cada instrucción en orden. Finalmente, retorna la suma, que es el resultado de la operación. Todo este proceso se lleva a cabo dentro del software que se ejecuta en la CPU, donde cada operación depende del reloj del procesador del ordenador que ejecuta el programa.

**Lenguaje de descripción de hardware (Verilog):**

```c
module suma (input [3:0] a, b, output [3:0] s);
    assign s = a + b;
endmodule
```
Como se puede observar en este código, se define un módulo de hardware llamado suma con dos entradas a y b de 4 bits cada una, y una salida s también de 4 bits. La instrucción assign s = a + b no se ejecuta secuencialmente como ocurre en los lenguajes de programación tradicionales. En cambio, esta declaración genera un circuito físico (implementado en FPGA o chip ASIC) compuesto por compuertas lógicas que realizan la suma de manera continua y paralela.
Cada vez que cambie la señal de entrada a o b, la salida s se actualiza automáticamente gracias al comportamiento combinacional del circuito, sin necesidad de ejecutar un programa secuencial ni depender de un reloj de procesador.

**2.**

| **Criterio** | **Microcontrolador (MCU)** | **Microprocesador (CPU)** | **FPGA (Field Programmable Gate Array)** | **PLD (Programmable Logic Device)** |
|---------------|-----------------------------|-----------------------------|-------------------------------------------|------------------------------------|
| **Definición** | Circuito integrado que contiene procesador, memoria y periféricos en un solo chip. | Unidad central de procesamiento que ejecuta instrucciones, pero depende de otros componentes externos. | Dispositivo programable que permite implementar circuitos digitales personalizados mediante lógica configurable. | Dispositivo lógico programable más simple que la FPGA, usado para implementar funciones lógicas básicas. |
| **Función principal** | Controlar sistemas embebidos y tareas específicas. | Ejecutar programas de propósito general. | Crear o prototipar hardware digital a nivel lógico. | Implementar circuitos lógicos combinacionales o secuenciales sencillos. |
| **Arquitectura interna** | Tiene CPU, memoria RAM/ROM y periféricos integrados (ADC, PWM, UART, etc.). | Solo incluye la unidad de procesamiento (ALU, registros, control). | Matriz de bloques lógicos interconectables configurables por el usuario. | Contiene compuertas lógicas y flip-flops programables. |
| **Tipo de programación** | Lenguaje de programación (C, C++, ensamblador). | Lenguaje de programación (C, Python, ensamblador). | Lenguaje de descripción de hardware (VHDL, Verilog). | Lenguaje de descripción de hardware (VHDL, ABEL, CUPL). |
| **Forma de ejecución** | Secuencial (una instrucción a la vez). | Secuencial y de alto rendimiento. | Paralela (varias operaciones simultáneas). | Paralela, pero limitada en complejidad. |
| **Velocidad de operación** | Moderada (MHz). | Alta (GHz). | Alta, depende de la configuración (decenas a cientos de MHz). | Media, adecuada para tareas lógicas simples. |
| **Flexibilidad** | Limitada al hardware integrado. | Muy flexible en software, pero hardware fijo. | Muy flexible, hardware reconfigurable. | Limitada, solo se puede programar una vez o pocas veces. |
| **Aplicaciones típicas** | Control de motores, sensores, robots, electrodomésticos. | Computadoras, servidores, smartphones. | Sistemas digitales personalizados, procesamiento paralelo, control avanzado. | Decodificadores, contadores, controladores lógicos sencillos. |
| **Ejemplo** | Arduino (ATmega328), ESP32, PIC16F877A. | Intel Core i7, ARM Cortex-A, AMD Ryzen. | Xilinx Spartan, Intel Cyclone, Altera MAX. | GAL, PAL, CPLD (versión avanzada de PLD). |

Como se evidencia en la tabla anterior, el microprocesador es un circuito integrado que combina procesamiento, memoria y periféricos en un solo chip, diseñado para controlar tareas específicas dentro de un sistema. EL microcontrolador es el cerebro de un sistema de cómputo, pero generalmente necesita memorias y periféricos externos para funcionar completamente. La FPGA es un dispositivo programable que permite crear circuitos digitales personalizados. A diferencia de un microprocesador, su hardware se puede configurar para ejecutar múltiples operaciones en paralelo, ofreciendo mayor flexibilidad y rendimiento en aplicaciones específicas. Por último, los PLD son dispositivos mucho más simples que las FPGA, utilizados para implementar pequeñas funciones lógicas como decodificadores o contadores. En conclusión se puede decir que los microcontroladores y microprocesadores se basan principalmente en software ejecutado secuencialmente, mientras que las FPGA y PLD se enfocan en hardware reconfigurable para diseñar sistemas digitales personalizados con capacidades de procesamiento paralelo.

**3.** 
Los lenguajes más utilizados en la programación de hardware son los lenguajes de descripción de hardware (HDL), que permiten modelar y simular el funcionamiento de circuitos digitales. Los más comunes son:

**VHDL (VHSIC Hardware Description Language):** muy usado en entornos académicos e industriales. Permite describir sistemas digitales de forma estructurada y precisa.

**Verilog:** más similar al lenguaje C, por lo que resulta más intuitivo para quienes vienen de la programación tradicional. Es ampliamente utilizado en el diseño de FPGA y ASIC.

**SystemVerilog:** una versión más avanzada de Verilog que agrega características de verificación y modelado de sistemas complejos.

Además, en niveles más altos de abstracción, también se usan herramientas basadas en lenguajes de alto nivel como SystemC (basado en C++) o MATLAB/Simulink, que permiten diseñar y generar código HDL automáticamente.

**4.**

### Proceso microcontrolado (ESP32)

Esta propuesta consiste en desarrollar un sistema embebido basado en el microcontrolador ESP32, que utiliza un sensor ultrasónico HC-SR04 para medir la distancia de un objeto frente al sistema. Con base en esa distancia, la velocidad del motor DC se regula mediante una señal PWM (Modulación por Ancho de Pulso), que controla un driver L293N encargado de alimentar el motor con la corriente adecuada. El sistema también cuenta con supervisión en el Monitor Serial, donde se visualizan los valores de distancia y velocidad, permitiendo verificar el correcto funcionamiento del control.

**Objetivo**
Diseñar e implementar un sistema que controle automáticamente la velocidad de un motor DC en función de la distancia de un obstáculo, garantizando protección cuando el objeto esté demasiado cerca y optimizando la respuesta del motor.

**Componentes:**
*ESP32 – Microcontrolador principal encargado de procesar la información.

*Sensor ultrasónico HC-SR04 – Mide la distancia entre el sensor y el objeto.

*Driver de motor L293N – Interfaz entre el microcontrolador y el motor DC; permite manejar mayores corrientes.

*Motor DC – Actuador que varía su velocidad según la distancia.

*Fuente de alimentación externa – Suministra energía al motor (separada del ESP32 para evitar sobrecargas).

*Cables, resistencias y protoboard – Para realizar las conexiones del circuito.

**Arquitectura**

HC-SR04 (TRIG/ECHO)
        ↓
      ESP32
        ↓
  PWM + Dirección
        ↓
   Driver de Motor
        ↓
      Motor DC

El sensor ultrasónico envía y recibe señales para medir la distancia. El ESP32 procesa esa información y genera una señal PWM que regula la velocidad del motor. El driver L293N actúa como un amplificador de potencia, ya que el ESP32 no puede entregar suficiente corriente directamente al motor.

**Funcionalidad propuesta**

*Modo de seguridad: Si la distancia medida es menor a 10 cm, el sistema detiene completamente el motor para evitar colisiones o daños.

*Modo lento: Si la distancia está entre 10 cm y 30 cm, el ESP32 genera una señal PWM de aproximadamente 50% de ciclo útil, haciendo que el motor gire a media velocidad.

*Modo rápido: Si la distancia es mayor a 30 cm, la señal PWM se establece al 100%, permitiendo que el motor funcione a su máxima velocidad.

*Monitoreo: Los valores de distancia y velocidad se muestran por el Monitor Serial, lo que facilita la supervisión y calibración del sistema.

*Medidas de seguridad adicionales: Se implementa un timeout en la lectura del sensor para detectar fallos o desconexiones.

*Si el sensor no responde, el motor se apaga por seguridad.


### Programación de hardware (Basys / FPGA)

En esta segunda propuesta, el mismo principio de control de distancia y velocidad se implementa mediante programación de hardware en una FPGA Basys3 (Artix-7). Aquí no se utiliza un microcontrolador tradicional, sino circuitos digitales diseñados directamente en lenguaje HDL (Verilog o VHDL), que se ejecutan de forma paralela y determinista.

El sistema genera internamente el pulso TRIG para activar el sensor HC-SR04, mide el tiempo del pulso ECHO con alta precisión y, a partir de ese tiempo, calcula la distancia. Según el valor obtenido, ajusta una señal PWM que controla la velocidad del motor a través de un driver externo.

**Objetivo**
Construir un sistema en hardware digital programable (FPGA) capaz de medir distancia en tiempo real con alta precisión y controlar la velocidad de un motor DC mediante una señal PWM generada de forma determinista.

**Componentes**

*Placa Basys3 (Artix-7) – FPGA que ejecuta los módulos de control en paralelo.

*Sensor HC-SR04 – Sensor ultrasónico para medir distancia.

*Driver de motor (L293N) – Amplificador de potencia para el motor.

*Motor DC y fuente de alimentación – Elemento mecánico a controlar.

*Adaptador de niveles TTL – Protege la FPGA de señales de 5V del sensor.

*Cable JTAG y software Vivado – Para programar y simular la FPGA.

**Arquitectura**

Módulo TRIG Generator → HC-SR04 → Módulo ECHO Counter
                     ↓
              Conversión a Distancia
                     ↓
             Comparador de Rangos
                     ↓
                Generador PWM
                     ↓
                Driver → Motor

Cada bloque del diagrama es un módulo independiente dentro de la FPGA:

-TRIG Generator: Genera un pulso de activación cada cierto tiempo (por ejemplo, cada 60 ms).

-ECHO Counter: Cuenta los ciclos de reloj que dura la señal ECHO en alto, representando el tiempo que tarda el sonido en ir y volver.

-Conversión a distancia: Convierte el conteo de ciclos en centímetros.

-Comparador de rangos: Determina si el objeto está cerca, medio o lejos.

-Generador PWM: Produce una señal PWM con el ciclo útil correspondiente (0%, 50% o 100%).

**Funcionalidad**

*Medición periódica: El sistema envía un pulso TRIG cada 60 milisegundos para iniciar una nueva medición.

*Conteo de tiempo: Mientras el pin ECHO esté en alto, un contador aumenta con el reloj del sistema (100 MHz), logrando una alta resolución temporal.

*Control del motor:
-Distancia < 10 cm → PWM = 0% (motor detenido).
-10–30 cm → PWM = 50% (velocidad media).
-30 cm → PWM = 100% (velocidad máxima).

*Indicadores LED: Cada rango de distancia enciende un LED distinto para mostrar el estado actual del sistema.

*Comunicación opcional: Si se implementa un módulo UART, la FPGA puede enviar datos al PC para registrar distancia y velocidad.

**5.**

###  Proceso microcontrolado
**Código (Arduino IDE)**
```cpp
#define TRIG 5
#define ECHO 18
#define IN1 14
#define IN2 27
#define ENA 26

long duration;
float distance;

void setup() {
  Serial.begin(115200);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  ledcSetup(0, 5000, 8);   // Canal 0, frecuencia 5 kHz, resolución 8 bits
  ledcAttachPin(ENA, 0);
  Serial.println("Sistema iniciado...");
}

void loop() {
  // Medición de distancia
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  
  duration = pulseIn(ECHO, HIGH, 25000); // Timeout 25 ms
  distance = duration * 0.034 / 2;

  // Control del motor según distancia
  if (distance < 10 || duration == 0) {
    detenerMotor();
  } else if (distance <= 30) {
    moverMotor(128);  // Velocidad media
  } else {
    moverMotor(255);  // Velocidad máxima
  }

  // Mostrar datos
  Serial.print("Distancia: ");
  Serial.print(distance);
  Serial.println(" cm");
  delay(200);
}

void moverMotor(int velocidad) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  ledcWrite(0, velocidad);
}

void detenerMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  ledcWrite(0, 0);
}

```

**Hallazgos**

-El sistema responde rápidamente ante la detección de obstáculos.

-Se observó una pequeña variación (±1 cm) en la medición del sensor, debido a interferencias o superficie del objeto.

-El control PWM permite transiciones suaves de velocidad.

-La comunicación serial facilita el monitoreo del sistema.

###  Programacion de hardware

**Código Base (Verilog)**
```cpp

module Control_Ultrasonico (
    input clk,
    input echo,
    output reg trig,
    output reg [7:0] pwm_out
);
    reg [31:0] counter;
    reg [31:0] echo_time;

    // Generación del pulso TRIG cada 60ms
    always @(posedge clk) begin
        if (counter < 3000) trig <= 1;
        else trig <= 0;
        counter <= (counter < 3_000_000) ? counter + 1 : 0;
    end

    // Conteo del pulso ECHO
    always @(posedge clk) begin
        if (echo) echo_time <= echo_time + 1;
        else echo_time <= 0;
    end

    // Generación PWM según distancia estimada
    always @(posedge clk) begin
        if (echo_time < 1000)
            pwm_out <= 0;         // Motor apagado
        else if (echo_time < 5000)
            pwm_out <= 128;       // Velocidad media
        else
            pwm_out <= 255;       // Velocidad máxima
    end
endmodule

```
**Hallazgos**

-El tiempo de respuesta es altamente preciso y determinista.

-El sistema funciona de manera completamente paralela, sin depender de software.

-Permite escalar fácilmente a varios sensores y motores simultáneamente.

-La depuración requiere más tiempo y experiencia en HDL.


| **Criterio** | **Sistema con ESP32 (Proceso Microcontrolado)** | **Sistema con FPGA (Programación de Hardware - Basys3)** |
|---------------|-----------------------------------------------|-----------------------------------------------------------|
| **Ventajas** | - Fácil programación mediante Arduino IDE.<br>- Permite agregar conectividad Wi-Fi o Bluetooth para monitoreo remoto.<br>- Económico y con amplia documentación disponible.<br>- Salida PWM precisa para control de motores. | - Ejecución determinista y completamente paralela.<br>- Alta precisión en el conteo del pulso ECHO.<br>- Escalable a varios sensores o motores en paralelo.<br>- Ideal para procesamiento en tiempo real y aprendizaje de diseño digital. |
| **Desventajas** | - Menor precisión temporal que un sistema basado en FPGA.<br>- Depende del software: si el microcontrolador se bloquea, el sistema se detiene.<br>- Limitado a tareas secuenciales (sin procesamiento paralelo). | - Mayor complejidad de diseño y depuración.<br>- Costo más alto que el ESP32.<br>- Menor flexibilidad para modificar el código (requiere regenerar el bitstream).<br>- Se necesitan conocimientos de HDL y herramientas de síntesis (Vivado). |


