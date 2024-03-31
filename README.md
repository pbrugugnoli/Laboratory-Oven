# Controlled and Monitored Laboratory Oven

## Design

- Design of a Laboratory Oven - Temperature Control for Food Experiments

	- The laboratory ovens, "estufa" (PT), are constructed with a Styrofoam box capable of accommodating up to 12 Petri Dishes in a thermally controlled environment.

	- Inside each box, a temperature sensor is installed within the lid, connected to a microcontroller positioned outside, in a controlling box. Additionally, a heater element is installed at the bottom of the box, and its power is controlled by the same microcontroller.

	- The microcontroller compares the target temperature provided with the temperature data received from the sensor. It employs a simple proportional control strategy (P), commonly known as "bang-bang", to determine the duty cycle of the Pulse Width Modulation (PWM) signal transmitted to the L298N Driver (H motor driver).

	- The L298N Driver regulates the power supplied to the heater elements in each box, adjusting the average voltage between 0V and 5V. To aid in the temperature stabilization within each box, a thermo-gel bag is placed on top of the heater elements. This thermo-gel bag serves several purposes:
		1. It ensures even distribution of heat at the bottom of the boxes.
		2. It promotes the equalization of heat throughout the box through convection waves.
		3. Due to its high temperature coefficient, the thermo-gel bag helps maintain the temperature inside the box even when it is open, particularly on colder days."
		
- "X-Ray diagram" of the laboratory ovens (boxes)
	![alt text](https://github.com/pbrugugnoli/Laboratory-Oven/blob/master/Images/XRAY.png?raw=true)

- Bill-of-Material (BOM)
	- Styrofoam box:  160 x 150 x 115 (external) 140 x 130 x 80 (internal)
		- [Mercado Livre](https://produto.mercadolivre.com.br/MLB-2102867731-3-caixa-de-isopor-1-litro-1-kg-multiuso-medicamentos-sorvete-_JM)
		- Investment: R$ 23,64 for 3 boxes
	- Silicon bed heater (mug heater kit)
		- Diameter: 9cm/3.54in (aprox.)  
		- Cable length: 110cm/43.31in (aprox.)  
		- Connector: USB  
		- Voltage: DC 5V  
		- Current 750MA  
		- Power: 3.75W
		- [Mercado Livre](https://produto.mercadolivre.com.br/MLB-3165666745-kit-aquecedor-xicara-caneca-usb-cafe-cha-leite-base-portatil-_JM)
		- Investment: 2 x R$ 24,90 = R$49,80
	- Dissipater (termogel bag)
		- [Mercado Livre](https://produto.mercadolivre.com.br/MLB-3681768652-bolsa-termica-com-gel-quente-fria-100gr-p-pequenas-lesoes-_JM?variation=#reco_item_pos=0&reco_backend=univb-vip-buybox&reco_backend_type=low_level&reco_client=vip-v2p&reco_id=19277ce7-42fd-48ab-8f34-c18bd452fcaf)
		- Investment: 2 x R$ 14,80 = R$ 25,60
	- Petri Dishes (sterilezed)
		- Diameter: 6,0 cm 
		- Height:  1,5 cm
		- [Mercado Livre](https://produto.mercadolivre.com.br/MLB-1603934261-placa-de-petri-60x15mm-ps-lisa-esteril-pacote-10-unidades-_JM)
		- Investment: 2 x R$ 11,90 = R$ 23,80 for 20 plates
	- Electronics (from my own stock of components - not purchase for the experiment)
		- NODEMCU ESP12E
			- [Mercado Livre](https://produto.mercadolivre.com.br/MLB-1211973212-modulo-wifi-esp8266-nodemcu-esp-12e-_JM#position=1&search_layout=stack&type=item&tracking_id=cdffab4f-46a7-4fca-97a6-1d4579cff972)
			- Investment: R$ 28,83
		- 2 x DHT11 - Temperature and Humidity sensor
			- [Mercado Livre](https://produto.mercadolivre.com.br/MLB-3472612190-sensor-umidade-temperatura-dht11-_JM#is_advertising=true&position=3&search_layout=grid&type=pad&tracking_id=7adacaa7-d237-4ec8-a9f9-e31c125b8adc&is_advertising=true&ad_domain=VQCATCORE_LST&ad_position=3&ad_click_id=ZDQ0MGQzNjktYTA0ZS00MmZlLWE2MGYtMDA2MGQ1YjA0Yzhj)
			- Investment: 2 x R$ 19,10 = R$ 38,20
		- 1 x L298N Driver - Module for controlling the heater devices
			- [Mercado Livre](https://produto.mercadolivre.com.br/MLB-2174675843-driver-motor-ponte-h-dupla-l298n-controladora-driver-l298n-_JM#is_advertising=true&position=1&search_layout=stack&type=pad&tracking_id=6cb07d7c-8c59-42fd-9dca-7c09c70f58aa&is_advertising=true&ad_domain=VQCATCORE_LST&ad_position=1&ad_click_id=N2QzZDk1NjItZWYwMC00MDc5LWE1YzMtOTljYmY0NDFhYjM0)
			- Investment: R$ 25,37
		- 1 x USB Charger with 2 USB outlets
			- [Mercado Livre](https://www.mercadolivre.com.br/carregador-de-parede-compact-2x-usb-21-a-10-w-geonav-esacb2-cor-preto/p/MLB20804926?pdp_filters=item_id:MLB3287977089#is_advertising=true&searchVariation=MLB20804926&position=1&search_layout=stack&type=pad&tracking_id=6992cc75-455d-44be-b4db-c96ce3f9d93b&is_advertising=true&ad_domain=VQCATCORE_LST&ad_position=1&ad_click_id=MmE1NTIxOWMtNTRhYi00MjVhLWJlYTUtOGQ1YzE0Mzc0NDUw)
			- Investment: R$32,90
		- Connecting Cables

## Build
- Software Configuration
	- Setup NODEMCU ESP12E to log data into google spreadsheet
		- Setup Arduino IDE 2.2.1
		- Setup ESP using https://blog.eletrogate.com/nodemcu-esp12-usando-arduino-ide-2/
		- Setup Logging procedure using https://github.com/StorageB/Google-Sheets-Logging
		- Setup L298N Driver - https://microcontrollerslab.com/l298n-dc-motor-driver-module-esp8266-nodemcu/
	- Setup POST and GET  SOAP services within the google sheet (as Web App):
			- ESP12E -> POST  target, current temperature and current duty cycle -> Sheets
			- ESP12E <- GET target temperatures and delay <- Sheets 
			- https://docs.google.com/spreadsheets/d/11LWHlFNlHQVi8g2V9y9Qfw_WbB1ok5Yn8jtM2-29634/edit#gid=344482888
			- NODEMCU ESP12E Program: ![[NODEMCU_ESP12E_2_channel_temp_controller_V01.ino]]
- Power Configuration 
	- Note that L298N has a 2V drop from the VCC to the output. Therefore, the 5V/2.5A power supply can only produce 3.0V in the output 
		- Heater elements 
			- -> 5V = 0.75A x R -> R = 6,66 Ohms
			- Power = (5V)^2 / R = 3,75W
		- A 12V/1A charger is also not good enough because because I would have to limit the duty cycle to 33% -> (12V - 2V) x (50% x 33%) = 3,33V  --->> 3.33 V / 6.66Ohm = 0.5A (charger limitation)
		- Let´s use the GoldenSky Power Source with a max duty cycle of 50% -> output (12V-2V) * 50% = 5V (~R$ 30,00)



- Physical build:
	- System Overview	- 	![alt text](https://github.com/pbrugugnoli/Laboratory-Oven/blob/master/Images/Laboratory-Ovens-(estufas)-Overview-2.jpeg?raw=true)
	
	- Controller Electronics (NODEMCU ESP12E + L298N Driver) ![alt text](https://github.com/pbrugugnoli/Laboratory-Oven/blob/master/Images/Laboratory-Ovens-(estufas)-Controller-System-Zoom.jpeg?raw=true)
	
	- Dashboard and Data Log on cloud (google cloud) ![alt text](https://github.com/pbrugugnoli/Laboratory-Oven/blob/master/Images/Laboratory-Ovens-(estufas)-Controller-System-Dashboard.jpeg?raw=true)
	
	- Temperature Sensos & Thermogel ![alt text](https://github.com/pbrugugnoli/Laboratory-Oven/blob/master/Images/Laboratory-Ovens-(estufas)-Temperature-Sensor-and-Thermogel.jpeg?raw=true)

	- Heater Element (adaptation with tea silicon heater) ![alt text](https://github.com/pbrugugnoli/Laboratory-Oven/blob/master/Images/Laboratory-Ovens-(estufas)-Heater-Element.jpeg?raw=true)

 ## Review 1
 - The solution based on HTTP Redirect to save data directly from the NODEMCU to Google Sheets turned out to be problematic causing unpredictable crashes of the NODEMCU. This crashes are really dangerous because the PWM signals got a 100% duty cycle, providing 12v to the 5V heater elements, burning them out (luckily there were some wood with alluminum foil to isolate them from the styrofoam boxes)
 - After many, many attempts to nail down the origen of the crashes (without any success), the program was converted to publish data directly to a InfluxDB hosted in a docker container of a desktop computer. It turned out to be a great solution as InfluxDB 2.0 has already a web interface and dashboards, but with 3 drawbacks:  (1) the desktop has to stay on as a server, (2) I couldn't manage to install InfluxDB 2.0 on a Raspberry PI 1B (32bit arm), and (3) to get access to the data when I am out of home without a proper VPN/security in place the data published to the InfluxDB triggers a task to write them down into the original Google Sheets (a quite interesting solution from a technical point of view and proof of concept, but quite ugly from a process stand point) - see branch InfluxDB-Logging
   
 ## Review 2
 - Without a short term solution for a local logging server, the program was changed once more to post that into a public MQQT (Adafruit), that turned out to be a great solution - simple code, data access from everywhere, ... but limited to 10 feeds. - see branch Adafruit-Logging

 ## Review 3
 - Even though no more system crashes were observed after Review 1 and 2, it cannot be completely ruled out and a solution to avoid burn down my house had to be implemented. One alternative is to change the power supply to limit the voltage to 5 volts (a 6V/7V power supply, for example, to compensate for the 1V/2V voltage drop of the L298N. Another option, more general and implemented here, is a supervisor circuit that monitors the average voltage and shuts down the power provided to the L298N (hence to the heater elements).
- The developed supervisor circuit consists of:
  - low pass filter + adjustable voltage comparator
  - latch circuit (once the voltage exceeds the defined limit, the circuit does not turn on until a push button is pressed to reset the circuit)
  - a mosfet switch output
- supervisor circuit  ![alt text](https://github.com/pbrugugnoli/Laboratory-Oven/blob/master/Images/Supervisor-Circuit.jpeg?raw=true)
- circuit desing and simulation http://tinyurl.com/ymkj8vpk
