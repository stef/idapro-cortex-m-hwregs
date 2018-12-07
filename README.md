# STM32f2xx HW register setup for IDA Pro.

This project contains a C header file that can be parsed by IDA to
define the structures of the STM32F2xx HW registers, as well as a tool
to generate an IDC script that creates segments for these HW registers
and sets their type to the appropriate struct. The addresses are
derived from the official CMSIS header files and the whole project
should be easily adaptable to other Cortex-M class MCUs.

## Usage

 1. Load your binary in IDA
 2. Set up your compiler (Options -> Compiler...) - the GNU C compiler is a good choice
 3. Load the `structs.h` file using (File -> Load File -> Parse C header file)
 4. Sync the newly imported types (View -> Open Subviews -> Local types) and then (Edit -> Synchronize to IDB)
 5. Verify you have a bunch of new structures in the Structures subview
 6. Run the stm32f2xx.idc script (File -> Script File)
 7. enjoy your properly set up HW registers.
