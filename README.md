## Cahce Simulator Introduction 

This project implements a cache simulator on an STM32 L476RG NUCLEO board. The simulator is designed to receive memory address information over UART and use direct memory access(DMA) for efficient data handling. It then checks whether the requested bytes are present in the simulated cache and returns cache statistics. The purpose of this project was to explore DMA and its uses. It was also created to understand how a low level cache works. 

## Operation 

Upon initialization, the program waits for a list of 12 byte chunks representing addresses intpretation information followed by a list of addresses to be sent over UART as a text file. The program will then check each of these addresses against an initially empty cache, storing duplicate addresses if possible. For each address sent, the program will send its intpretation of the address(see details below), and finally a summary of cache statistics once all addresses have been received. 

Available Cache Settings: 
  + Word Size: 1 to 4 bytes
  + Memory Size: 1024 to 4294967296 bytes
  + Total Cahce Size: 256 to 32Kb
  + Cache Block Size: 16 to 4096 bytes

Example Input: 

0x00000004 Number of addresses in the sequence  
0x00001000 Size of main memory  
0x00000100 Size of cache memory  
0x00000010 Size of cache block  
0x00000001 Word Size  
0x00000AAA Address 1  
0x00000BBB Address 2  
0x00000CCC Address 3  
0x00000AAA Address 4  

The addreass intpretation output is formatted as follows: "memory address - tag value - index value - block offset value - hit/miss" 

Example Output From the First Address: 
0x00000AAA - 0x000A - 0x000A - 0x000A - miss 

Example Final Statistics Output: 

Memory address lines = 12 
Tag bits = 4 
Index bits = 4 
Offset bits = 4 
Hit rate = 1/4 = 25% 

## Running 
All code was created for the L476RG NUCLEO board utilizing the STM Cube IDE. The code should work for other NUCELO boards, provided a different .ioc file is created. Code entry point may be found in Core/Src/main.c. Simply import the project into the Cube IDE and right click on it to build. Then connect your NUCLEO board over UART and hit the run button. Note: A third party program will need to be used to send text files over UART, the build in UART sender is not recommended but may be used. 

## FInal Notes 
All code has been created for educational purposes only and is not inteded for production use. 


