## To flash the firmware:

1. Connect board to PC.
2. The new device should show up under ports in Device Manager:

    ![image](https://github.com/user-attachments/assets/76dcb878-7901-40a3-a7db-b9b6b6a256d2)
3. Visit: https://espressif.github.io/esptool-js/
4. Click 'Connect'
5. Select the COM port from step 2 and click 'Connect.'
6. Enter 0 in 'Flash Address' and click 'Choose File' and select the firmware.bin file.
7. Click 'Program'
8. Wait until the output shows 'Hash of data verified. Leaving...'
9. Power cycle the board and the new firmware should be loaded.
