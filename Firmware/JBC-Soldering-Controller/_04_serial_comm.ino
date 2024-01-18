/* Copyright (c) 2017 timothyjager
   JBC-Soldering-Controller
   MIT License. See LICENSE file for details.
*/

//----------------Serial Comm------------------------

//process serial communications
//this function shoudl be called cylcilcallly in the main loop
#define SERIAL_COMM_PERIOD_MS 250 //run every 200ms.
void ProcessSerialComm(void)
{
  static long next_millis = millis() + SERIAL_COMM_PERIOD_MS;  //determine the next time this function should activate
  static bool serial_active = false;

  //send-receive with processing if it's time
  if (millis() > next_millis)
  {
    //Check if there is data on the serial port. This indicated the host PC is sending us something.
    if (Serial.available())
    {
      //try to read and process the serial commands
      /*if (SerialReceive())
      {
        serial_active = true;
      }*/
    }
    //Once we determine there is serial activity, we assume a host is connected, so begin streaming our packet data to the host. TODO: currently no way to detect if the host dissapears.
    if (Serial)
    {
      //SendStatusPacket();
      //Serial.print(0.03125 * status.adc_ic_temp_counts);
      //Serial.print(status.adc_ic_temp_counts);
      //Serial.print("\t\t");
      //Serial.println(status.tip_temperature_c);
      //double tip_temp_c = ADS.ADCcode2temp(status.adc_counts + ADS.localCompensation(status.adc_ic_temp_counts));
      Serial.print(status.tip_temperature_c);
      Serial.print("\t");
      Serial.print(status.adc_counts);
      Serial.print("\t");
      Serial.print(status.adc_ic_temp_counts/128.0);
      Serial.print("\t");
      Serial.print(ADS.localCompensation(status.adc_ic_temp_counts));
      Serial.print("\t");
      Serial.println(ADS.ADCcode2temp(status.adc_counts)/10.0);
    }
    next_millis += SERIAL_COMM_PERIOD_MS;  //set up our loop to run again in x ms
  }
}



//this function sends all of our status values as a hex string. this reduces the CPU load on the MCU since it doesnt have to format float strings.
void SendStatusPacket()
{
  /*
    controller_packet.status.start_of_packet = 0xBA;
    controller_packet.status.setpoint = Setpoint;
    noInterrupts(); //make sure we disable interrupts while grabing these volatile values.
    controller_packet.status.input = Input;
    controller_packet.status.output = Output;
    //controller_packet.status.ITerm = myPID.GetITerm();
    interrupts();
    controller_packet.status.kP = myPID.GetKp();
    controller_packet.status.kI = myPID.GetKi();
    controller_packet.status.kD = myPID.GetKd();
    controller_packet.status.automatic = (myPID.GetMode() == AUTOMATIC);
    controller_packet.status.simulate_input = host_packet.param.simulate_input;
  */

  //Update GPIO status.
  status.gpio_port_b = PINB;
  status.gpio_port_c = PINC;
  status.gpio_port_d = PIND;
  status.gpio_port_e = PINE;
  /*
    if (params.idle_temp_c > 1)
    {
    status.encoder_pos = 1;           //Enocder Position
    status.adapter_voltage_mv = 2;    //Input power adapter voltage in millivolts
    status.adc_counts = 3;            //ADC value read by ADS1118
    status.adc_ic_temp_counts = 4;    //internal temp of ADS1118
    status.current_sense_mv = 5;      //current sense in milliamps
    status.pid_setpoint = 33000;            //setpoint of the PID loop
    status.pid_input = 32000;               //input value of the PID loop
    status.pid_output = -12999;              //computed output value of the PID loop


    params.automatic = 1;        //PID mode - Automatic=0, Manual=1
    params.simulate_input = 0;   //this allows us to override the actual input (temperature reading) using the tuning app
    //params.idle_temp_c = 123;
    params.setpoint = 1.23;
    params.kP = 2.34;
    params.kI = 4.56;
    params.kD = 6.78;
    params.SimulatedInput = 9.101;
    }
  */
  //Copy status and parameter structures into our packet payload
  noInterrupts();
  memcpy(&controller_packet.payload.status, &status, sizeof(status_struct));
  memcpy(&controller_packet.payload.params, &params, sizeof(system_parameters_struct));
  interrupts();

  //send all of the bytes in the controller_
  //Serial.write("BA");  //Start of packet indicator. TODO: change this to non-hex code
  for (int i = 0; i < sizeof(controller_packet_struct); i++)
  {
    //convert each byte into a hex string.
    const char lookup[] = "0123456789abcdef";
    Serial.write(lookup[ controller_packet.asBytes[i] >> 4 ]);
    Serial.write(lookup[ controller_packet.asBytes[i] & 0x0f ]);
  }
  Serial.write('\n');
}

//Receive commands from the tuning PC app
bool SerialReceive()
{
  bool return_value = false;
  // read the bytes sent from tuning app
  int index = 0;
  //if there are bytes in the serial buffer, then read them until we have received a full packet
  while (Serial.available() && index < sizeof(host_packet_struct))
  {
    host_packet.asBytes[index] = Serial.read();
    index++;
  }

  //when there are no more bytes to read, check if we read the correct number of bytes and make sure our first byte equals our hard-coded start of packet value
  if (index == sizeof(host_packet_struct) && host_packet.payload.start_of_packet == 0xAB)
  {
    noInterrupts();
    memcpy(&params, &host_packet.payload.params, sizeof(status_struct));  //copy our payload struct into our param struct

    //Update PID settings
    myPID.SetMode(params.pid_mode);
    //if we are in manual mode, override the output  
    if (params.pid_mode == MANUAL)
    {
      status.pid_output = params.output_override;
    }
    else
    {
    status.pid_setpoint = params.setpoint;      
    }
    myPID.SetTunings(params.kP, params.kI, params.kD);

    knob.write(params.setpoint);
    interrupts();
    
    return_value = true;
  }
  Serial.flush();                         // * clear any random data from the serial buffer
  return return_value;
}
