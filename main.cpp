/*
  @date 2023/10/04
  @author Naito Masaki
  @brief ArduCAMをmbedで動かすためのコード

  @reference
  https://os.mbed.com/users/justinkim/code/DigitalCamera_OV5642_WIZwiki-W7500/
  https://github.com/ArduCAM/Arduino.git
*/

#include "mbed.h"
#include "ov5642.h"
#include "ov5642_regs.h"

#include <stdio.h>
#include <string.h>

#include "HEPTA_CDH.h"
#include "HEPTA_EPS.h"

#define PHOTO_RESOLUTION 4 // FHD
#define LOOP_TIMES 3  // how many pictures you take
#define JPEG_SIZE OV5642_640x480

#define LOW_POWER 0
#define NORMAL_POWER 1
#define POWER_MODE LOW_POWER

ArduCAM myCAM(p5, p6, p7, p21, p28, p27);
HEPTA_CDH cdh(p5, p6, p7, p8, "sd");
HEPTA_EPS eps(p16,p26);

char fnamecamera[32];
char fnamecntcamera = 0;

/* Functions*/
void check_wiring();
void initalize_device();
void take_picture();

int main()
{
  eps.turn_on_regulator();
  printf("\nArduCAM Start!\r\n");
  check_wiring();
  initalize_device();
  for (int count = 0; count < LOOP_TIMES; count++)
  {
    printf("=====  Take %d =====\r\n", count);
    take_picture();
  }
  printf("all sequence finished\r\n");
}

void check_wiring()
{
  // usb device identification
  uint8_t vid, pid;

  // write 0x55 to test register to confirm wiring is collect
  while (1)
  {
    myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
    // delay is needed after writing register to reflect it
    wait_ms(200);
    uint8_t temp = myCAM.read_reg(ARDUCHIP_TEST1);
    printf("temp:0x%x\n", temp);

    if (temp != 0x55)
    {
      printf("SPI interface Error!\r\n");
    }
    else
    {
      printf("SPI interface OK.\r\n");
      break;
    }
  }

  // Reset the CPLD
  // this is needed for arducam plus series
  myCAM.write_reg(0x07, 0x80);
  wait_ms(100);
  myCAM.write_reg(0x07, 0x00);
  wait_ms(100);

  // Check if the camera module type is OV5642
  myCAM.wrSensorReg16_8(0xff, 0x01);
  myCAM.rdSensorReg16_8(OV5642_CHIPID_HIGH, &vid);
  myCAM.rdSensorReg16_8(OV5642_CHIPID_LOW, &pid);
  if ((vid != 0x56) || (pid != 0x42))
  {
    printf("Can't find OV5642 module!\r\n");
  }
  else
  {
    printf("OV5642 detected\r\n");
  }
}

void initalize_device()
{
  // set capture 1 photo per time
  myCAM.write_reg(CAPTURE_CTRL_REG, 0x00);
  wait_ms(200);
  uint8_t capture_control_reg = myCAM.read_reg(CAPTURE_CTRL_REG);
  printf("Capture Control Register: %d\r\n", capture_control_reg);

  if (POWER_MODE == LOW_POWER)
  {
    // Change to BMP capture mode and initialize the OV5642 module
    // so that decrease consumption of electric power
    myCAM.set_format(BMP);
    myCAM.InitCAM();
  }
  else
  {
    myCAM.set_format(JPEG);
    myCAM.InitCAM();
    myCAM.write_reg(ARDUCHIP_TIM, VSYNC_LEVEL_MASK); // VSYNC is active HIGH
    myCAM.OV5642_set_JPEG_size(OV5642_1920x1080);
  }

  wait_ms(500);
}

void take_picture()
{
  uint8_t buf[256];
  static int i = 0;
  uint8_t temp = 0;
  uint8_t temp_last = 0;

  if (POWER_MODE == LOW_POWER)
  {
    myCAM.write_reg(CAPTURE_CTRL_REG, 0x00);
    myCAM.set_format(JPEG);
    wait_ms(200);
    myCAM.InitCAM();
    myCAM.write_reg(ARDUCHIP_TIM, VSYNC_LEVEL_MASK); // VSYNC is active HIGH
    myCAM.OV5642_set_JPEG_size(JPEG_SIZE);
    wait_ms(1000);
  }

  // Flush the FIFO
  myCAM.flush_fifo();
  // Clear the capture done flag
  myCAM.clear_fifo_flag();
  // Start capture
  myCAM.start_capture();
  printf("Start Capture\r\n");

  while (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK))
    ;
  printf("Capture Done.\r\n");

  uint32_t length = myCAM.read_fifo_length();
  printf("The fifo length is : %u \r\n", length);

  if (POWER_MODE == LOW_POWER)
  {
    myCAM.set_format(BMP);
    myCAM.InitCAM();
    wait_ms(500);
  }

  // // Construct a file name
  // char fileName[24];
  // get_file_name(fileName);
  snprintf(fnamecamera, sizeof(fnamecamera), "/sd/jpss%04d.jpg", fnamecntcamera);
  fnamecntcamera++;
  // Open the new file
  FILE *outFile = fopen(fnamecamera, "w");

  if (outFile == NULL)
  {
    printf("open file failed\r\n");
    wait_ms(500);
    return;
  }
  printf("file opend\r\n");

  bool is_header = false;

  myCAM.set_fifo_burst();
  while (length--)
  {
    temp_last = temp;
    temp = myCAM.read_fifo();
    // Write image data to buffer if not full
    // Read JPEG data from FIFO
    if ((temp == 0xD9) && (temp_last == 0xFF)) // If find the end ,break while,
    {
      buf[i++] = temp; // save the last  0XD9
      // Write the remain bytes in the buffer
      fwrite(buf, i, 1, outFile);
      // Close the file
      fclose(outFile);
      printf("Image save OK.\r\n");
      is_header = false;
      i = 0;
      break;
    }
    if (is_header == true)
    {
      // Write image data to buffer if not full
      if (i < 256)
      {
        buf[i++] = temp;
      }
      else
      {
        // Write 256 bytes image data to file
        fwrite(buf, 256, 1, outFile);
        i = 0;
        buf[i++] = temp;
        myCAM.set_fifo_burst();
      }
    }
    else if ((temp == 0xD8) & (temp_last == 0xFF))
    {
      is_header = true;
      buf[i++] = temp_last;
      buf[i++] = temp;
    }
  }

  printf("file save finished\r\n");
}