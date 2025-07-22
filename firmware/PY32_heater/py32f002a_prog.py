#!/usr/bin/env python3
# ===================================================================================
# Project:   py32f002a_prog - Tool for PY32F002A Microcontroller 
# Version:   v2.7
# Year:      2025
# Author:    Based on puyaisp by Stefan Wagner, adapted and optimized by Vladimir Trondin
# Purpose:   Connects to PY32F002A via USB-to-serial to read chip parameters, read/write
#            memory, erase memory, start application, manage read/write protection,
#            or test actual flash memory size.
# License:   MIT License
# ===================================================================================

import sys
import time
import serial
import argparse
import os
import configparser
from typing import Optional, Dict, Any

# ===================================================================================
# Configuration Definitions
# ===================================================================================

# Default configuration with user-friendly names
DEFAULT_CONFIG = {
    'serial_port': '/dev/ttyUSB0',           # Serial port for USB-to-serial adapter
    'baud_rate': '115200',                   # Baud rate (4800 - 1000000, default: 115200)
    'read_file': 'eeprom.bin',               # Default file for reading memory
    'write_file': 'main.bin',                # Default file for writing memory
    'flash_size_kb': '20',                   # Flash memory size in kilobytes
    'reset_active': 'low',                   # Active level for reset (low/high/off)
    'boot_active': 'high',                   # Active level for boot mode (low/high/off)
    'reset_pin': 'rts',                      # Pin used for reset (rts/dtr/off)
    'boot_pin': 'dtr',                       # Pin used for boot mode (rts/dtr/off)
    'auto_bootloader': 'true'               # Automatically enter bootloader mode for operations
}

# Load or create configuration from .ini file
def load_config():
    """Load configuration from .ini file or create one with defaults."""
    config = configparser.ConfigParser()
    script_name = os.path.splitext(os.path.basename(sys.argv[0]))[0]
    config_file = f"{script_name}.ini"
    
    if os.path.exists(config_file):
        config.read(config_file)
        if 'DEFAULT' not in config:
            config['DEFAULT'] = DEFAULT_CONFIG
    else:
        config['DEFAULT'] = DEFAULT_CONFIG
        with open(config_file, 'w') as configfile:
            config.write(configfile)
    
    # Assign configuration values
    global PY_PORT, PY_BAUD, PY_DEFAULT_READ_FILE, PY_DEFAULT_WRITE_FILE, PY_FLASH_SIZE, PY_AUTO_BOOTLOADER
    PY_PORT = config['DEFAULT']['serial_port']
    PY_BAUD = int(config['DEFAULT']['baud_rate'])
    PY_DEFAULT_READ_FILE = config['DEFAULT']['read_file']
    PY_DEFAULT_WRITE_FILE = config['DEFAULT']['write_file']
    PY_FLASH_SIZE = int(config['DEFAULT']['flash_size_kb']) * 1024  # Convert KB to bytes
    PY_AUTO_BOOTLOADER = config['DEFAULT'].getboolean('auto_bootloader')
    return config

# Load configuration at startup
config = load_config()

# ===================================================================================
# Signal Control Constants
# ===================================================================================

# Signal states
SIGNAL_ACTIVE = True
SIGNAL_INACTIVE = False

# Pin modes
PIN_RTS = 'rts'
PIN_DTR = 'dtr'
PIN_OFF = 'off'

# ===================================================================================
# Timing Diagrams for Reset and Boot Mode Entry
# ===================================================================================
#
# Normal Operation:
#   Reset:    ---------------------------- (high/inactive)
#   Boot:     ---------------------------- (low/inactive)
#
# Hardware Reset (500ms pulse):
#   Reset:    _____|---------------------- (active low)
#   Boot:     ---------------------------- (unchanged)
#
#   OR (if active high):
#   Reset:    -----|______________________ (active high)
#   Boot:     ---------------------------- (unchanged)
#
# Boot Mode Entry Sequence:
#   1. Reset goes active (500ms)
#   2. Boot goes active during reset (500ms)
#   3. Reset goes inactive
#   4. Boot goes inactive after another 500ms
#
# Example (active low reset, active high boot):
#   Reset:    _____|----------------------
#   Boot:     ------------|_______________
#
# Example (active high reset, active low boot):
#   Reset:    -----|______________________
#   Boot:     _____________|--------------
# ===================================================================================

# Other configuration constants
PY_MAX_TEST_SIZE = 48 * 1024      # Maximum test size for flash memory (48 KB)
PY_TEST_BLOCKSIZE = 1024          # Test block size (1 KB)

# Device and memory constants
PY_CHIP_PID     = 0x440           # Expected PID for PY32F002A
PY_BLOCKSIZE    = 128             # Flash block size in bytes
PY_FLASH_ADDR   = 0x08000000      # Flash memory start address
PY_CODE_ADDR    = 0x08000000      # Code start address
PY_SRAM_ADDR    = 0x20000000      # SRAM start address
PY_BOOT_ADDR    = 0x1fff0000      # Bootloader start address
PY_UID_ADDR     = 0x1fff0e00      # Unique ID address
PY_OPTION_ADDR  = 0x1fff0e80      # Option bytes address
PY_CONFIG_ADDR  = 0x1fff0f00      # Configuration address

# Command codes
PY_CMD_GET      = 0x00            # Get bootloader info
PY_CMD_VER      = 0x01            # Get version
PY_CMD_PID      = 0x02            # Get product ID
PY_CMD_READ     = 0x11            # Read memory
PY_CMD_WRITE    = 0x31            # Write memory
PY_CMD_ERASE    = 0x44            # Erase memory
PY_CMD_GO       = 0x21            # Go (start application)
PY_CMD_W_LOCK   = 0x63            # Write lock
PY_CMD_W_UNLOCK = 0x73            # Write unlock
PY_CMD_R_LOCK   = 0x82            # Read lock
PY_CMD_R_UNLOCK = 0x92            # Read unlock

# Reply codes
PY_REPLY_ACK    = 0x79            # Acknowledge
PY_REPLY_NACK   = 0x1f            # Negative acknowledge
PY_REPLY_BUSY   = 0xaa            # Busy

# Other codes
PY_SYNCH        = 0x7f            # Synchronization byte

# Default option bytes
PY_OPTION_DEFAULT = b'\xaa\xbe\x55\x41\xff\x00\x00\xff\xff\xff\xff\xff\xff\xff\x00\x00'

# ===================================================================================
# Programmer Class (Optimized)
# ===================================================================================

class PY32Programmer:
    """Optimized class to handle communication with PY32F002A microcontroller."""
    
    def __init__(self, port: str = PY_PORT):
        """Initialize with port configuration."""
        self.port = port
        self.baudrate = PY_BAUD
        self.serial: Optional[serial.Serial] = None
        self.ver: Optional[int] = None
        self.verstr: Optional[str] = None
        self.pid: Optional[int] = None
        self.option: Optional[bytes] = None
        self.optionstr: Optional[str] = None
        self.uid: Optional[bytes] = None
        self.debug: bool = False
        self.skip_sync = False
        self.auto_bootloader = PY_AUTO_BOOTLOADER
        
        # Load reset/boot configuration
        self.reset_active = config['DEFAULT'].get('reset_active', 'low').lower()
        self.boot_active = config['DEFAULT'].get('boot_active', 'high').lower()
        self.reset_pin = config['DEFAULT'].get('reset_pin', 'rts').lower()
        self.boot_pin = config['DEFAULT'].get('boot_pin', 'dtr').lower()
        
        # Validate configuration
        if self.reset_active not in ('low', 'high', 'off'):
            self.reset_active = 'low'
        if self.boot_active not in ('low', 'high', 'off'):
            self.boot_active = 'high'
        if self.reset_pin not in ('rts', 'dtr', 'off'):
            self.reset_pin = 'rts'
        if self.boot_pin not in ('rts', 'dtr', 'off'):
            self.boot_pin = 'dtr'

    def __enter__(self):
        """Context manager entry."""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()

    def _set_signal_state(self, pin: str, state: bool) -> None:
        """Set the state of a specified pin (RTS/DTR).
        
        Args:
            pin: Pin to control (PIN_RTS/PIN_DTR/PIN_OFF)
            state: Desired state (SIGNAL_ACTIVE/SIGNAL_INACTIVE)
        """
        if not self.serial or pin == PIN_OFF:
            return
            
        if pin == PIN_RTS:
            self.serial.rts = state
        elif pin == PIN_DTR:
            self.serial.dtr = state

    def _get_signal_state(self, pin: str, active_level: str) -> bool:
        """Determine the signal state based on active level.
        
        Args:
            pin: Pin to check (PIN_RTS/PIN_DTR/PIN_OFF)
            active_level: Active level configuration ('low'/'high'/'off')
            
        Returns:
            bool: True for active state, False for inactive
        """
        if pin == PIN_OFF or active_level == 'off':
            return SIGNAL_INACTIVE
        return active_level == 'low'

    def _set_reset_state(self, active: bool) -> None:
        """Set reset pin to specified state.
        
        Args:
            active: Whether to set reset to active state
        """
        desired_state = SIGNAL_ACTIVE if active else SIGNAL_INACTIVE
        if self.reset_active == 'high':
            desired_state = not desired_state
        self._set_signal_state(self.reset_pin, desired_state)

    def _set_boot_state(self, active: bool) -> None:
        """Set boot pin to specified state.
        
        Args:
            active: Whether to set boot to active state
        """
        desired_state = SIGNAL_ACTIVE if active else SIGNAL_INACTIVE
        if self.boot_active == 'low':
            desired_state = not desired_state
        self._set_signal_state(self.boot_pin, desired_state)

    def _initialize_pins(self) -> None:
        """Initialize all control pins to their inactive states."""
        self._set_reset_state(False)
        self._set_boot_state(False)
        time.sleep(0.1)  # Let signals stabilize

    def connect(self, skip_sync: bool = False) -> None:
        """Establish connection and optionally synchronize with the bootloader."""
        self.skip_sync = skip_sync
        try:
            # Initialize serial port with RTS/DTR in inactive state
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                parity=serial.PARITY_EVEN,
                timeout=2,
                rtscts=False,
                dsrdtr=False
            )
            
            # Initialize all control pins
            self._initialize_pins()
            
            # Clear any existing data
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            
            # Skip synchronization if requested (for hardware operations)
            if not self.skip_sync:
                # If auto-bootloader is enabled, enter bootloader mode first
                if self.auto_bootloader:
                    self.enter_bootloader_mode()
                
                # Synchronize with bootloader
                if self.debug:
                    print(f"DEBUG: Sending SYNC byte: 0x{PY_SYNCH:02x}")
                self.serial.write([PY_SYNCH])
                if not self._check_reply():
                    raise ConnectionError("Synchronization failed")
                
        except serial.SerialException as e:
            self.close()
            raise ConnectionError(f"Failed to open port {self.port}: {str(e)}")

    def close(self) -> None:
        """Safely close the serial connection."""
        if self.serial and self.serial.is_open:
            try:
                # Set pins to inactive state before closing
                self._initialize_pins()
                self.serial.close()
            except:
                pass
        self.serial = None
        
  
        

    def hardware_reset(self) -> None:
        """Perform hardware reset using configured reset pin and active level."""
        if not self.serial or self.reset_pin == PIN_OFF:
            return
            
        # Initialize to inactive state
        self._initialize_pins()
        time.sleep(0.1)
        # Set reset to active level
        self._set_reset_state(True)
        
        # Hold reset for 500ms
        time.sleep(0.5)
        
        # Set reset to inactive level
        self._set_reset_state(False)
        time.sleep(0.2)

    def enter_bootloader_mode(self) -> None:
        """Enter bootloader mode using configured reset and boot pins."""
        if not self.serial:
            return
            
        # Initialize both pins to inactive state
        self._initialize_pins()
        
        # Step 1: Activate reset
        self._set_reset_state(True)
        
        # Step 2: Activate boot pin during reset
        time.sleep(0.1)  # Short delay between signals
        self._set_boot_state(True)
        
        # Hold both for 500ms
        time.sleep(0.5)
        
        # Step 3: Release reset
        self._set_reset_state(False)
        
        # Step 4: Release boot after another 500ms
        time.sleep(0.5)
        self._set_boot_state(False)

    def _send_command(self, command: int) -> None:
        """Send a command and check for acknowledgment."""
        if not self.serial:
            raise ConnectionError("Serial port not open")
            
        if self.debug:
            print(f"DEBUG: Sending command: 0x{command:02x}, checksum: 0x{command ^ 0xff:02x}")
        self.serial.write(bytes([command, command ^ 0xff]))
        if not self._check_reply():
            raise IOError(f"Command 0x{command:02x} not acknowledged")

    def _send_address(self, addr: int) -> None:
        """Send a 32-bit address with parity."""
        if not self.serial:
            raise ConnectionError("Serial port not open")
            
        stream = addr.to_bytes(4, byteorder='big')
        parity = 0x00
        for byte in stream:
            parity ^= byte
        if self.debug:
            print(f"DEBUG: Sending address: 0x{addr:08x}, bytes: {stream.hex()}, parity: 0x{parity:02x}")
        self.serial.write(stream + bytes([parity]))
        if not self._check_reply():
            raise IOError(f"Address not acknowledged")

    def _check_reply(self, reply: bytes = None) -> bool:
        """Check if the device sent an ACK reply."""
        if not self.serial:
            return False
            
        if reply is None:
            reply = self.serial.read(1)
        if self.debug:
            if reply:
                if reply[0] == PY_REPLY_ACK:
                    print(f"DEBUG: Received reply: 0x{reply.hex()} (ACK)")
                elif reply[0] == PY_REPLY_NACK:
                    print(f"DEBUG: Received reply: 0x{reply.hex()} (NACK)")
                else:
                    print(f"DEBUG: Received reply: 0x{reply.hex()} (Unknown)")
            else:
                print("DEBUG: Received reply: none")
        return reply and reply[0] == PY_REPLY_ACK

    def _read_info_stream(self, command: int) -> bytes:
        """Read an info stream after sending a command."""
        self._send_command(command)
        size = self.serial.read(1)[0]
        if self.debug:
            print(f"DEBUG: Info stream size: {size}")
        stream = self.serial.read(size + 1)
        if self.debug:
            print(f"DEBUG: Info stream: {stream.hex()}")
        if not self._check_reply():
            raise IOError("Failed to read info stream")
        return stream

    def read_chip_info(self) -> None:
        """Read bootloader version and product ID."""
        self.ver = self._read_info_stream(PY_CMD_GET)[0]
        self.verstr = f"{self.ver >> 4}.{self.ver & 7}"
        self.pid = int.from_bytes(self._read_info_stream(PY_CMD_PID), byteorder='big')

    def read_uid(self) -> None:
        """Read the 128-byte unique ID and store it."""
        self.uid = self.read_flash(PY_UID_ADDR, 128)

    def read_option_bytes(self) -> None:
        """Read and format option bytes."""
        try:
            self.option = self.read_flash(PY_OPTION_ADDR, 16)
            optr = (self.option[0] << 8) + self.option[1]
            sdkr = (self.option[4] << 8) + self.option[5]
            wrpr = (self.option[12] << 8) + self.option[13]
            self.optionstr = f"OPTR: 0x{optr:04x}, SDKR: 0x{sdkr:04x}, WRPR: 0x{wrpr:04x}"
        except Exception as e:
            raise IOError(f"Failed to read option bytes (chip may be locked): {str(e)}")

    def read_flash(self, addr: int, size: int) -> bytes:
        """Read data from flash memory."""
        if not self.serial:
            raise ConnectionError("Serial port not open")
            
        data = bytearray()
        while size > 0:
            blocksize = min(size, PY_BLOCKSIZE)
            try:
                self._send_command(PY_CMD_READ)
                self._send_address(addr)
                self._send_command(blocksize - 1)
                block = self.serial.read(blocksize)
                if len(block) != blocksize:
                    raise IOError(f"Failed to read {blocksize} bytes at 0x{addr:08x}")
                data.extend(block)
            except Exception as e:
                raise IOError(f"Read error at 0x{addr:08x}: {str(e)}")
            addr += blocksize
            size -= blocksize
        return bytes(data)

    def read_memory(self, address: int = PY_FLASH_ADDR, size: int = PY_FLASH_SIZE, filename: str = PY_DEFAULT_READ_FILE) -> None:
        """Read memory from the specified address and save to file."""
        print(f"Reading {size} bytes from address 0x{address:08x} to {filename}...")
        data = self.read_flash(address, size)
        with open(filename, 'wb') as f:
            f.write(data)
        print(f"SUCCESS: {len(data)} bytes read and saved to {filename}.")

    def write_memory(self, address: int, filename: str, no_reset: bool = False) -> None:
        """Write file to memory, erase first, verify, and optionally start application.
        
        Args:
            address: Memory address to write to
            filename: File to write
            no_reset: If True, don't start application after writing
        """
        if not os.path.exists(filename):
            raise IOError(f"File {filename} does not exist")
        with open(filename, 'rb') as f:
            data = f.read()
        if not data:
            raise IOError(f"File {filename} is empty")
        if all(b == 0xFF for b in data):
            raise IOError(f"File {filename} contains only 0xFF bytes")
        
        print(f"Erasing flash memory...")
        self.erase_memory()
        time.sleep(0.1)
        
        self.read_option_bytes()
        wrpr = (self.option[12] << 8) + self.option[13]
        if wrpr != 0xFFFF:
            raise IOError(f"Write protection active (WRPR: 0x{wrpr:04x}). Unlock with -wu first.")
        
        print(f"Writing {len(data)} bytes from {filename} to address 0x{address:08x}...")
        size = len(data)
        addr = address
        while size > 0:
            blocksize = min(size, PY_BLOCKSIZE)
            if blocksize % 4 != 0:
                padding = bytes([0xFF] * (4 - (blocksize % 4)))
                block = data[addr - address:addr - address + blocksize] + padding
                blocksize += len(padding)
            else:
                block = data[addr - address:addr - address + blocksize]
            
            self._send_command(PY_CMD_WRITE)
            self._send_address(addr)
            self.serial.write(bytes([blocksize - 1]))
            self.serial.write(block)
            parity = blocksize - 1
            for b in block:
                parity ^= b
            self.serial.write(bytes([parity]))
            if not self._check_reply():
                raise IOError(f"Failed to write block at 0x{addr:08x}")
            
            addr += blocksize
            size -= blocksize
        
        print(f"Verifying written data...")
        verify_data = self.read_flash(address, len(data))
        if verify_data != data:
            raise IOError("Verification failed: Written data does not match file")
        
        if not no_reset:
            print(f"Starting application at 0x{address:08x}...")
            self.go(address)
            print(f"SUCCESS: {len(data)} bytes written, verified, and application started.")
        else:
            print(f"SUCCESS: {len(data)} bytes written and verified (no reset).")

    def erase_memory(self) -> None:
        """Erase flash memory."""
        print("Erasing flash memory...")
        self._send_command(PY_CMD_ERASE)
        self.serial.write(b'\xff\xff\x00')
        time.sleep(0.1)
        if not self._check_reply():
            raise IOError("Failed to erase memory")
        print("SUCCESS: Flash memory erased.")

    def go(self, address: int) -> None:
        """Start application at the specified address."""
        self._send_command(PY_CMD_GO)
        self._send_address(address)
        print(f"SUCCESS: Application started at 0x{address:08x}.")

    def write_lock(self) -> None:
        """Lock flash memory against writing (WARNING: Use with caution)."""
        print("WARNING: Write lock may prevent further writes until unlocked. Proceed at your own risk.")
        self._send_command(PY_CMD_W_LOCK)
        if not self._check_reply():
            raise IOError("Failed to lock write protection")
        print("SUCCESS: Flash memory write-locked.")

    def write_unlock(self) -> None:
        """Unlock flash memory for writing."""
        self._send_command(PY_CMD_W_UNLOCK)
        if not self._check_reply():
            raise IOError("Failed to unlock write protection")
        print("SUCCESS: Flash memory write-unlocked.")

    def read_lock(self) -> None:
        """Lock flash memory against reading (WARNING: Use with caution)."""
        print("WARNING: Read lock may prevent reading until unlocked. Proceed at your own risk.")
        self._send_command(PY_CMD_R_LOCK)
        if not self._check_reply():
            raise IOError("Failed to lock read protection")
        print("SUCCESS: Flash memory read-locked.")

    def read_unlock(self) -> None:
        """Unlock flash memory for reading."""
        self._send_command(PY_CMD_R_UNLOCK)
        if not self._check_reply():
            raise IOError("Failed to unlock read protection")
        print("SUCCESS: Flash memory read-unlocked.")

    def test_memory(self, max_size: int = PY_MAX_TEST_SIZE) -> None:
        """Test flash memory size by writing and verifying 1 KB blocks."""
        print(f"Testing flash memory up to {max_size // 1024} KB...")
        print("WARNING: This test will erase and write the entire flash memory. Proceed with caution.")
        
        self.erase_memory()
        time.sleep(0.1)
        
        self.read_option_bytes()
        wrpr = (self.option[12] << 8) + self.option[13]
        if wrpr != 0xFFFF:
            raise IOError(f"Write protection active (WRPR: 0x{wrpr:04x}). Unlock with -wu first.")
        
        verified_size = 0
        addr = PY_FLASH_ADDR
        while addr < PY_FLASH_ADDR + max_size:
            kb_offset = (addr - PY_FLASH_ADDR) // 1024
            print(f"Writing at 0x{addr:08x} ({kb_offset} KB)...")
            
            # Write 1 KB with unique 32-bit pattern (address-based)
            block = bytearray()
            for i in range(PY_TEST_BLOCKSIZE // 4):
                block.extend((addr + i * 4).to_bytes(4, byteorder='big'))
            try:
                size = len(block)
                block_addr = addr
                while size > 0:
                    blocksize = min(size, PY_BLOCKSIZE)
                    if blocksize % 4 != 0:
                        padding = bytes([0xFF] * (4 - (blocksize % 4)))
                        subblock = block[block_addr - addr:block_addr - addr + blocksize] + padding
                        blocksize += len(padding)
                    else:
                        subblock = block[block_addr - addr:block_addr - addr + blocksize]
                    
                    self._send_command(PY_CMD_WRITE)
                    self._send_address(block_addr)
                    self.serial.write(bytes([blocksize - 1]))
                    self.serial.write(subblock)
                    parity = blocksize - 1
                    for b in subblock:
                        parity ^= b
                    self.serial.write(bytes([parity]))
                    if not self._check_reply():
                        raise IOError(f"Failed to write block at 0x{block_addr:08x}")
                    
                    block_addr += blocksize
                    size -= blocksize
            except Exception as e:
                print(f"Write error at 0x{addr:08x}: {str(e)}. Test stopped.")
                break
            
            # Verify all memory from 0x08000000 to current address
            print(f"Verifying up to 0x{addr + PY_TEST_BLOCKSIZE:08x} ({kb_offset + 1} KB)...")
            verify_addr = PY_FLASH_ADDR
            try:
                while verify_addr < addr + PY_TEST_BLOCKSIZE:
                    verify_size = min(PY_TEST_BLOCKSIZE, (addr + PY_TEST_BLOCKSIZE) - verify_addr)
                    data = self.read_flash(verify_addr, verify_size)
                    
                    # Generate expected data for verification
                    expected = bytearray()
                    for i in range(verify_size // 4):
                        expected.extend((verify_addr + i * 4).to_bytes(4, byteorder='big'))
                    if verify_size % 4:
                        expected.extend([0xFF] * (verify_size % 4))
                    
                    if data != expected:
                        raise IOError(f"Verification failed at 0x{verify_addr:08x}. Data mismatch.")
                    
                    verify_addr += verify_size
                verified_size = addr + PY_TEST_BLOCKSIZE - PY_FLASH_ADDR
            except Exception as e:
                print(f"Verification error: {str(e)}. Test stopped.")
                break
            
            addr += PY_TEST_BLOCKSIZE
        
        print(f"SUCCESS: Verified {verified_size} bytes ({verified_size // 1024} KB) of flash memory.")

    def test_ram(self) -> None:
        """Test RAM by reading 4 bytes every 256 bytes up to 8KB."""
        print("Testing RAM by reading 4 bytes every 256 bytes up to 8KB...")
        print("Address    | Data (4 bytes)")
        print("-----------|---------------")
        
        addr = PY_SRAM_ADDR
        end_addr = PY_SRAM_ADDR + 8 * 1024  # Test up to 8KB of RAM
        prev_kb = -1  # To track KB boundaries
        
        while addr < end_addr:
            current_kb = (addr - PY_SRAM_ADDR) // 1024
            # Print separator line when crossing KB boundary
            if current_kb != prev_kb:
                if prev_kb != -1:  # Don't print before first block
                    print("-----------|---------------")
                prev_kb = current_kb
            
            try:
                # Read 4 bytes at current address
                data = self.read_flash(addr, 4)
                print(f"0x{addr:08x} | {data.hex(' ', 1)}")
            except Exception as e:
                print(f"Error reading at 0x{addr:08x}: {str(e)}")
                break
            
            # Move to next address (increment by 256 bytes)
            addr += 256
        
        print("RAM test completed.")

    def get_parameters(self) -> Dict[str, Any]:
        """Retrieve chip parameters with formatted UID."""
        self.read_chip_info()
        self.read_uid()
        self.read_option_bytes()
        
        uid_dict = {}
        if self.uid:
            lot_number = self.uid[:8].decode('ascii', errors='replace')
            uid_dict["Lot Number"] = lot_number
            wafer_coords = self.uid[8:12].hex()
            uid_dict["Wafer/Coordinates"] = f"0x{wafer_coords}"
            serial_number = self.uid[12:24].hex()
            uid_dict["Serial Number"] = f"0x{serial_number}"
            remaining = self.uid[24:]
            if all(b == 0xFF for b in remaining):
                uid_dict["Remaining"] = "Reserved (all 0xFF)"
            else:
                remaining_hex = remaining.hex()
                chunk_size = 64
                chunks = [remaining_hex[i:i+chunk_size] for i in range(0, len(remaining_hex), chunk_size)]
                uid_dict["Remaining"] = "\n" + "\n".join(f"    0x{chunk}" for chunk in chunks)
        
        return {
            "Bootloader Version": self.verstr,
            "Product ID": f"0x{self.pid:04x}",
            "Unique ID": uid_dict,
            "Option Bytes": self.optionstr
        }

# ===================================================================================
# Main Function
# ===================================================================================

def main() -> None:
    """Main function to handle command-line arguments and execute operations."""
    parser = argparse.ArgumentParser(
        description="PY32F002A Microcontroller Programming Tool",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
Usage examples:
  python3 py32f002a_prog.py -i              # Read chip parameters
  python3 py32f002a_prog.py -r data.bin     # Read 20 KB to data.bin
  python3 py32f002a_prog.py -w app.bin      # Write app.bin to flash with reset
  python3 py32f002a_prog.py -hr             # Perform hardware reset
  python3 py32f002a_prog.py -hb             # Enter bootloader mode via hardware
  python3 py32f002a_prog.py -wnr app.bin    # Write app.bin to flash without reset
  python3 py32f002a_prog.py -e              # Erase flash memory
  python3 py32f002a_prog.py -g              # Start application
  python3 py32f002a_prog.py -wl             # Lock write protection (use with caution)
  python3 py32f002a_prog.py -t              # Test actual flash memory size
  python3 py32f002a_prog.py -tr             # Test RAM by reading 4 bytes every 256 bytes
  python3 py32f002a_prog.py -ab             # Enable auto-bootloader mode
  python3 py32f002a_prog.py -nab            # Disable auto-bootloader mode

Hardware Control:
  Reset and boot mode entry can be controlled via RTS/DTR pins. Configuration is
  stored in the ini file (reset_active, boot_active, reset_pin, boot_pin).
  Defaults: reset_active=low (RTS), boot_active=high (DTR).
"""
    )
    parser.add_argument('-i', '--info', action='store_true', help='Read and display chip parameters (default)')
    parser.add_argument('-r', '--read', nargs='?', const=PY_DEFAULT_READ_FILE, metavar='FILE', help='Read flash memory to file (default: eeprom.bin, 20 KB)')
    parser.add_argument('-w', '--write', nargs='?', const=PY_DEFAULT_WRITE_FILE, metavar='FILE', help='Write file to flash memory, erase first, verify, and start (default: main.bin)')
    parser.add_argument('-wnr', '--write-no-reset', nargs='?', const=PY_DEFAULT_WRITE_FILE, metavar='FILE', help='Write file to flash memory without resetting the MCU (default: main.bin)')
    parser.add_argument('-e', '--erase', action='store_true', help='Erase flash memory')
    parser.add_argument('-g', '--go', action='store_true', help='Start application at code address')
    parser.add_argument('-wl', '--write-lock', action='store_true', help='Lock flash memory against writing (WARNING: May prevent writes until unlocked)')
    parser.add_argument('-wu', '--write-unlock', action='store_true', help='Unlock flash memory for writing')
    parser.add_argument('-rl', '--read-lock', action='store_true', help='Lock flash memory against reading (WARNING: May prevent reads until unlocked)')
    parser.add_argument('-ru', '--read-unlock', action='store_true', help='Unlock flash memory for reading')
    parser.add_argument('-t', '--test-memory', action='store_true', help='Test actual flash memory size (up to 48 KB)')
    parser.add_argument('-tr', '--test-ram', action='store_true', help='Test RAM by reading 4 bytes every 256 bytes up to 8KB')
    parser.add_argument('-hr', '--hard-reset', action='store_true', help='Perform hardware reset using RTS/DTR pins')
    parser.add_argument('-hb', '--hard-boot', action='store_true', help='Enter bootloader mode using hardware reset sequence')
    parser.add_argument('-ab', '--auto-bootloader', action='store_true', help='Enable auto-bootloader mode')
    parser.add_argument('-nab', '--no-auto-bootloader', action='store_true', help='Disable auto-bootloader mode')

    args = parser.parse_args()

    # Handle auto-bootloader configuration changes
    if args.auto_bootloader or args.no_auto_bootloader:
        config['DEFAULT']['auto_bootloader'] = 'true' if args.auto_bootloader else 'false'
        with open(f"{os.path.splitext(os.path.basename(sys.argv[0]))[0]}.ini", 'w') as configfile:
            config.write(configfile)
        print(f"Auto-bootloader mode {'enabled' if args.auto_bootloader else 'disabled'}")
        sys.exit(0)

    if not any(vars(args).values()):
        parser.print_help()
        sys.exit(0)

    print(f"Connecting to PY32F002A via {PY_PORT}...")
    try:
        programmer = PY32Programmer()
        
        # For hardware operations, skip synchronization
        if args.hard_reset or args.hard_boot:
            programmer.connect(skip_sync=True)
        else:
            programmer.connect()
            
        print("SUCCESS: Connection established.")
        
        try:
            if args.info:
                print("\nReading chip parameters...")
                params = programmer.get_parameters()
                print("\nPY32F002A Parameters:")
                print("---------------------")
                for key, value in params.items():
                    if key == "Unique ID":
                        print(f"{key}:")
                        for subkey, subvalue in value.items():
                            if subkey == "Remaining" and subvalue.startswith("\n"):
                                print(f"  {subkey}:")
                                print(subvalue.replace("\n", "\n  "))
                            else:
                                print(f"  {subkey}: {subvalue}")
                    else:
                        print(f"{key}: {value}")
                if programmer.pid != PY_CHIP_PID:
                    print(f"\nWARNING: PID 0x{programmer.pid:04x} does not match expected PY32F002A!")
            
            if args.read:
                programmer.read_memory(filename=args.read)
            
            if args.write:
                programmer.write_memory(PY_FLASH_ADDR, args.write)
            
            if args.write_no_reset:
                programmer.write_memory(PY_FLASH_ADDR, args.write_no_reset, no_reset=True)
            
            if args.erase:
                programmer.erase_memory()
            
            if args.go:
                programmer.go(PY_CODE_ADDR)
            
            if args.write_lock:
                programmer.write_lock()
            
            if args.write_unlock:
                programmer.write_unlock()
            
            if args.read_lock:
                programmer.read_lock()
            
            if args.read_unlock:
                programmer.read_unlock()
            
            if args.test_memory:
                programmer.test_memory()
            
            if args.test_ram:
                programmer.test_ram()
            
            if args.hard_reset:
                programmer.hardware_reset()
                print("SUCCESS: Hardware reset performed.")
                sys.exit(0)
                
            if args.hard_boot:
                programmer.enter_bootloader_mode()
                print("SUCCESS: Bootloader mode entered via hardware sequence.")
                sys.exit(0)
                
        finally:
            programmer.close()
            print("\nDONE.")
            
    except Exception as e:
        sys.stderr.write(f"\nERROR: {str(e)}\n")
        sys.exit(1)

if __name__ == "__main__":
    main()
