/*

MinIMU-9-Arduino-AHRS
Pololu MinIMU-9 + Arduino AHRS (Attitude and Heading Reference System)

Copyright (c) 2011-2016 Pololu Corporation.
http://www.pololu.com/

MinIMU-9-Arduino-AHRS is based on sf9domahrs by Doug Weibel and Jose Julio:
http://code.google.com/p/sf9domahrs/

sf9domahrs is based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose
Julio and Doug Weibel:
http://code.google.com/p/ardu-imu/

MinIMU-9-Arduino-AHRS is free software: you can redistribute it and/or modify it
under the terms of the GNU Lesser General Public License as published by the
Free Software Foundation, either version 3 of the License, or (at your option)
any later version.

MinIMU-9-Arduino-AHRS is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for
more details.

You should have received a copy of the GNU Lesser General Public License along
with MinIMU-9-Arduino-AHRS. If not, see <http://www.gnu.org/licenses/>.

*/

void transmitData(void)
{   
      Bluetooth.write("!");

      #if write_EULER == 1
      Bluetooth.write("ANG:");
      Bluetooth.write(ToDeg(roll));
      Bluetooth.write(",");
      Bluetooth.write(ToDeg(pitch));
      Bluetooth.write(",");
      Bluetooth.write(ToDeg(yaw));
      #endif      
      #if write_ANALOGS==1
      Bluetooth.write(",AN:\t");
      Bluetooth.write(AN[0]);  //(int)read_adc(0)
      Bluetooth.write(",");
      Bluetooth.write(AN[1]);
      Bluetooth.write(",");
      Bluetooth.write(AN[2]);  
      Bluetooth.write(",\t");
      Bluetooth.write(AN[3]);
      Bluetooth.write (",");
      Bluetooth.write(AN[4]);
      Bluetooth.write (",");
      Bluetooth.write(AN[5]);
      Bluetooth.write(",\t");
      Bluetooth.write(c_magnetom_x);
      Bluetooth.write (",");
      Bluetooth.write(c_magnetom_y);
      Bluetooth.write (",");
      Bluetooth.write(c_magnetom_z);
      #endif
      #if write_DCM == 1
      Bluetooth.write (",DCM:");
      Bluetooth.write(DCM_Matrix[0][0]);
      Bluetooth.write (",");
      Bluetooth.write(DCM_Matrix[0][1]);
      Bluetooth.write (",");
      Bluetooth.write(DCM_Matrix[0][2]);
      Bluetooth.write (",");
      Bluetooth.write(DCM_Matrix[1][0]);
      Bluetooth.write (",");
      Bluetooth.write(DCM_Matrix[1][1]);
      Bluetooth.write (",");
      Bluetooth.write(DCM_Matrix[1][2]);
      Bluetooth.write (",");
      Bluetooth.write(DCM_Matrix[2][0]);
      Bluetooth.write (",");
      Bluetooth.write(DCM_Matrix[2][1]);
      Bluetooth.write (",");
      Bluetooth.write(DCM_Matrix[2][2]);
      #endif
      Bluetooth.write('\n');
      
}

/*long convert_to_dec(float x)
{
  return x*10000000;
}*/
