library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library ROrgPrSimLib;
use ROrgPrSimLib.proc_config.all;
use ROrgPrSimLib.flashROM;
use ROrgPrSimLib.flashRAM;

entity mipsCpu is
	generic(PROG_FILE_NAME : string;
			DATA_FILE_NAME : string);
	port(clk : in std_logic;
		 rst : in std_logic;

		 -- instruction insertion ports
		 testMode_debug : in std_logic;
		 testInstruction_debug : in std_logic_vector(31 downto 0);

		 -- ram access ports
		 ramInsertMode_debug : in std_logic;
		 ramWriteEn_debug : in std_logic;
		 ramWriteAddr_debug : in std_logic_vector(LOG2_NUM_RAM_ELEMENTS - 1 downto 0);
		 ramWriteData_debug : in std_logic_vector(RAM_ELEMENT_WIDTH - 1 downto 0);
		 ramElements_debug : out ram_elements_type;

		 -- register file access port
		 registers_debug : out reg_vector_type;

		 -- intermediate result ports
		 pc_next_debug : out std_logic_vector(PC_WIDTH - 1 downto 0);
		 pc7SegDigits_debug : out pc_7seg_digits_type
		 );
end mipsCpu;

architecture structural of mipsCpu is
signal invClk : std_logic;
signal PC_IN : std_logic_vector(PC_WIDTH - 1 downto 0) := "00000000000000000000000000000100";
signal TO_PC : std_logic_vector(PC_WIDTH - 1 downto 0) := "00000000000000000000000000000100";
signal PC_OUT: std_logic_vector(PC_WIDTH - 1 downto 0);
signal regDst, branch, memRead, memToReg, memWrite, aluSrc, regWrite, branchMux : std_logic;
signal aluOp : std_logic_vector(1 downto 0);
signal instruct, instr : std_logic_vector(RAM_ELEMENT_WIDTH - 1 downto 0);
signal aluZero, toWriteEn : std_logic;
signal toRegWriteAddr : std_logic_vector(4 downto 0);
signal aluOperation : std_logic_vector(3 downto 0);
signal toAluB, shiftedNumber, mainAluResult, memReadData, vectSignExtNumber, toRegWriteData : std_logic_vector(REG_WIDTH - 1 downto 0);
signal  readD1, readD2, branchAluResult, addFourAluResult, toWriteData : std_logic_vector(REG_WIDTH - 1 downto 0);
signal four : std_logic_vector(REG_WIDTH - 1 downto 0) := "00000000000000000000000000000100";
signal signExtNumber : signed(REG_WIDTH - 1 downto 0);
signal signed_number : signed (15 downto 0);
signal toMemAddr : std_logic_vector(9 downto 0);
begin
	invClk <= not clk;
	-- PC Debug Ports
	Debug1: entity ROrgPrSimLib.bin2Char(behavioral)
		port map( bin => PC_OUT(3 downto 0),
			bitmask => pc7SegDigits_debug(0));
	Debug2: entity ROrgPrSimLib.bin2Char(behavioral)
		port map( bin => PC_OUT(7 downto 4),
			bitmask => pc7SegDigits_debug(1));
	Debug3: entity ROrgPrSimLib.bin2Char(behavioral)
		port map( bin => PC_OUT(11 downto 8),
			bitmask => pc7SegDigits_debug(2));
	Debug4: entity ROrgPrSimLib.bin2Char(behavioral)
		port map( bin => PC_OUT(15 downto 12),
			bitmask => pc7SegDigits_debug(3));

	pc_next_debug <= PC_IN;
	-- Input of Program Counter
	TO_PC <= PC_IN when testMode_debug = '0'
		else PC_OUT when testMode_debug = '1'
		else (others => 'X');

	-- Address of the next command
	PC_IN <= addFourAluResult when branchMux = '0'
		else branchAluResult when branchMux = '1'
		else (others => 'X');

	-- Address of Register to be written
	toRegWriteAddr <= instruct(20 downto 16) when regDst = '0' else instruct(15 downto 11)
			  when regDst = '1'
			else (others => 'X');

	-- B signal to ALU
	toAluB <= readD2 when aluSrc = '0' else
		std_logic_vector(signExtNumber) when aluSrc = '1'
		else (others => 'X');

	-- Data which go to RegisterWrite port
	toRegWriteData <= memReadData when memToReg = '1'
			else mainAluResult when memToReg = '0'
			else (others => 'X');

	-- Address of RAM to be written
	toMemAddr <=  mainAluResult(11 downto 2) when ramInsertMode_debug = '0'
			else ramWriteAddr_debug when ramInsertMode_debug = '1'
			else (others => 'X');

	-- WriteEnable of RAM
	toWriteEn <= memWrite when ramInsertMode_debug = '0' else ramWriteEn_debug
			when ramInsertMode_debug = '1'
			else 'X';

	-- Data which go to RAM write port
	toWriteData <= readD2 when ramInsertMode_debug = '0'
			else ramWriteData_debug when ramInsertMode_debug = '1'
			else (others => 'X');

	-- Control signal of a "branch or PC+4" muxer
	branchMux <= aluZero and branch;


	-- MIPS Control Unit
	MIPS_CTRL: entity ROrgPrSimLib.mipsCtrl(structural)
	port map(op => instruct(31 downto 26),
		 regDst => regDst,
		 branch => branch,
		 memRead => memRead,
		 memToReg => memToReg,
		 aluOp => aluOp,
		 memWrite => memWrite,
		 aluSrc => aluSrc,
		 regWrite => regWrite);

	-- Program Counter
	PROGRAM_COUNTER: entity ROrgPrSimLib.reg(behavioral)
		generic map(WIDTH => PC_WIDTH)
		port map(clk => clk,
			rst => rst,
			en => '1',
			D => TO_PC,
			Q => PC_OUT);

	-- Instruction Memory
	INSTR_ROM: entity ROrgPrSimLib.flashROM(behavioral)
		generic map(NUM_ELEMENTS => NUM_ROM_ELEMENTS,
			LOG2_NUM_ELEMENTS => 10,
			ELEMENT_WIDTH => RAM_ELEMENT_WIDTH,
			INIT_FILE_NAME => PROG_FILE_NAME)
		port map(address => PC_OUT(11 downto 2),
			readData => instr);


	-- TestMode instruction
	instruct <= instr when testMode_debug = '0'
	else testInstruction_debug when testMode_debug = '1'
	else (others => 'X');


	-- Registers
	REGISTERS: entity ROrgPrSimLib.regFile(structural)
		generic map(NUM_REGS => NUM_REGS,
			LOG2_NUM_REGS => LOG2_NUM_REGS,
			REG_WIDTH => REG_WIDTH)
		port map(clk => clk,
		 	rst => rst,
		 	readAddr1 => instruct(25 downto 21),
		 	readData1 => readD1,
		 	readAddr2 => instruct(20 downto 16),
		 	readData2 => readD2,
		 	writeEn => regWrite,
		 	writeAddr => toRegWriteAddr,
		 	writeData => toRegWriteData,
		 	reg_vect_debug => registers_debug);

	-- Sign Extend
	signed_number <= signed(instruct(15 downto 0));
	signEx: entity ROrgPrSimLib.signExtend(behavioral)
		generic map(INPUT_WIDTH => 16,
			OUTPUT_WIDTH => 32)
		port map(number => signed_number,
			signExtNumber => signExtNumber);

	-- ALU Control Unit
	ALUCtrl: entity ROrgPrSimLib.aluCtrl(behavioral)
		port map(aluOp =>aluOp,
			f => instruct(5 downto 0),
			operation => aluOperation);

	-- Main ALU
	ALU: entity ROrgPrSimLib.csAlu(behavioral)
		generic map(WIDTH => REG_WIDTH)
		port map(ctrl => aluOperation,
		 	a => readD1,
		 	b => toAluB,
		 	result => mainAluResult,
			overflow => open,
			zero => aluZero);

	-- Data Memory
	DATA_RAM: entity ROrgPrSimLib.flashRAM(behavioral)
		generic map(NUM_ELEMENTS => NUM_RAM_ELEMENTS,
			LOG2_NUM_ELEMENTS => LOG2_NUM_RAM_ELEMENTS,
			ELEMENT_WIDTH => RAM_ELEMENT_WIDTH,
			INIT_FILE_NAME => DATA_FILE_NAME)
		port map(clk => invClk,
			address => toMemAddr,
			writeEn => toWriteEn,
			writeData => toWriteData,
			readEn => memRead,
			readData => MemReadData,
			ramElements_debug => ramElements_debug);

	-- Shift Left 2
	vectSignExtNumber <= std_logic_vector(signExtNumber);
	SL2: entity ROrgPrSimLib.leftShifter(behavioral)
		generic map( WIDTH => REG_WIDTH,
				SHIFT_AMOUNT => 2)
		port map(number => vectSignExtNumber,
			shiftedNumber => shiftedNumber);

	-- Branch ALU
	BRANCH_ALU: entity ROrgPrSimLib.csAlu(behavioral)
		generic map( WIDTH => REG_WIDTH)
		port map ( ctrl => "0010", -- 00|10 - neither aInv, nor bInv | addition
			a => PC_OUT,
			b => shiftedNumber,
			result => branchAluResult,
			overflow => open,
			zero => open);

	-- Add +4 ALU
	ADD4ALU:
		entity ROrgPrSimLib.csAlu(behavioral)
		generic map( WIDTH => REG_WIDTH)
		port map ( ctrl => "0010", -- 00|10 - neither aInv, nor bInv | addition
			a => PC_OUT,
			b => four,
			result => addFourAluResult,
			overflow => open,
			zero => open);
end structural;
