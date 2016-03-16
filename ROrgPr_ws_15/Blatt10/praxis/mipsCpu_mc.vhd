library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library ROrgPrSimLib;
use ROrgPrSimLib.proc_config.all;
use ROrgPrSimLib.flashRAM;

entity mipsCpu_mc is
	generic(INIT_FILE_NAME : string);
	port(clk : in std_logic;
		 rst : in std_logic;

		 -- debug ports
		 ramElements_debug : out ram_elements_type;
		 registers_debug : out reg_vector_type;
		 mipsCtrlState_debug : out mips_ctrl_state_type;
		 pc7SegDigits_debug : out pc_7seg_digits_type
		 );
end mipsCpu_mc;

architecture structural of mipsCpu_mc is

	--signal declarations
	signal sA, sB: std_logic_vector(REG_WIDTH-1 downto 0);
	signal saddress : std_logic_vector(REG_WIDTH - 1 downto 0);

	signal swriteregister : std_logic_vector(LOG2_NUM_REGS-1 downto 0);
	signal swritedata : std_logic_vector(REG_WIDTH -1 downto 0);
	signal saluA : std_logic_vector(REG_WIDTH -1 downto 0);
	signal saluB : std_logic_vector(REG_WIDTH -1 downto 0);

	-- Sign Extend
	signal sSignedNumber : signed(15 downto 0);
	signal sSignExtendedNumber : signed(REG_WIDTH -1 downto 0);

	-- Left shifter
	signal sVectSignExtend : std_logic_vector(REG_WIDTH -1 downto 0);
	signal sShiftedNumber : std_logic_vector(REG_WIDTH -1 downto 0);

	-- Other signals
	signal sPCen : std_logic;
	signal sPrintWriteConditionZero : std_logic;
	signal sInstruction : std_logic_vector(REG_WIDTH-1 downto 0);
	signal sShiftInstruction : std_logic_vector(27 downto 0);
	signal sJumpAddress : std_logic_vector(REG_WIDTH-1 downto 0);

	-- Four
	signal four : integer := 4;
	signal sSLVfour : std_logic_vector(REG_WIDTH-1 downto 0);

	-- RAM
	signal smemData : std_logic_vector(RAM_ELEMENT_WIDTH-1 downto 0)	;
	signal smemDatareg : std_logic_vector(RAM_ELEMENT_WIDTH-1 downto 0);

	-- Registers
	signal sReaddata1 : std_logic_vector(RAM_ELEMENT_WIDTH-1 downto 0);
	signal sReaddata2 : std_logic_vector(RAM_ELEMENT_WIDTH-1 downto 0);

	-- PC
	signal sPC : std_logic_vector(PC_WIDTH-1 downto 0) ;
	signal stoPC : std_logic_vector(PC_WIDTH-1 downto 0);

	-- ALU
	signal sALUresult : std_logic_vector(REG_WIDTH-1 downto 0);
	signal sALUout : std_logic_vector(REG_WIDTH-1 downto 0);

	-- MIPS Ctrl out signals
	signal sInvClk : std_logic;
	signal sregDst : std_logic;
	signal smemRead : std_logic;
	signal smemToReg : std_logic;
	signal saluOp : std_logic_vector(1 downto 0);
	signal smemWrite : std_logic;
	signal sregWrite : std_logic;
	signal saluSrcA : std_logic;
	signal saluSrcB : std_logic_vector(1 downto 0);
	signal spcSrc : std_logic_vector(1 downto 0);
	signal sirWrite : std_logic;
	signal sIorD : std_logic;
	signal spcWrite : std_logic;
	signal spcWriteCond : std_logic;

	-- ALU Ctrl out signal
	signal soperation : std_logic_vector(3 downto 0);
	signal szero : std_logic;

begin

	-- Beschreibung der Multicycle-MIPS-CPU hier ergänzen
	-- invert clock
	sInvClk <= not clk;

	-- populate sSLVfour vector
	sSLVfour <= std_logic_vector(to_signed(four,sSLVfour'length));

	-- set PC enable
	sPrintWriteConditionZero <= spcWriteCond and szero;
	sPCen <= sPrintWriteConditionZero or spcWrite;

	-- Sign extend
	sSignedNumber <= signed(sInstruction(15 downto 0));
	sVectSignExtend <= std_logic_vector(sSignExtendedNumber);

	-- Jump control
	sShiftInstruction <= sInstruction(25 downto 0) & (1 downto 0 => '0');
	sJumpAddress <= sPC(PC_WIDTH-1 downto 28) & sShiftInstruction;

	-- Program counter
	PROGRAM_COUNTER: entity ROrgPrSimLib.reg(behavioral)
		generic map(WIDTH => PC_WIDTH)
		port map(clk => clk,
						rst => rst,
						en => sPCen,
						D => stoPC,
						Q => sPC);

	saddress <=	sPC when sIorD = '0'
							else sALUout when sIorD = '1'
							else (others => 'X');

	-- Data and instruction memory
	INSTR_AND_DATA_RAM: entity ROrgPrSimLib.flashRAM(behavioral)
		generic map(NUM_ELEMENTS => NUM_RAM_ELEMENTS,
					LOG2_NUM_ELEMENTS => LOG2_NUM_RAM_ELEMENTS,
					ELEMENT_WIDTH => RAM_ELEMENT_WIDTH,
					INIT_FILE_NAME => INIT_FILE_NAME)
		port map(clk => sInvClk,
				 address => saddress(11 downto 2),
				 writeEn => smemWrite,
				 writeData => sB,
				 readEn => smemRead,
				 readData => smemData,
				 ramElements_debug => ramElements_debug);

	-- Registers
	REGS: entity ROrgPrSimLib.regFile(structural)
		generic map(NUM_REGS => NUM_REGS,
								LOG2_NUM_REGS => LOG2_NUM_REGS,
								REG_WIDTH => REG_WIDTH)
		port map(clk => clk,
							rst => rst,
							readAddr1 => sInstruction(25 downto 21),
							readData1 => sReaddata1,
							readAddr2 => sInstruction(20 downto 16),
							readData2 => sReaddata2,
							writeEn => sregWrite,
							writeAddr => swriteregister,
							writeData => swritedata,
							reg_vect_debug => registers_debug);

	-- ALU control
	ALU_CONTROL: entity ROrgPrSimLib.aluCtrl(behavioral)
		port map(aluOp => saluOp,
						f => sInstruction(5 downto 0),
						operation => soperation);

	-- ALU
	ALU: entity ROrgPrSimLib.csALU(behavioral)
		generic map(WIDTH => REG_WIDTH)
		port map(ctrl => soperation,
						a => saluA,
						b => saluB,
						result => sALUresult,
						overflow => open,
						zero => szero);

	-- MIPS control (finite state machine)
	MIPS_CONTROL_FSM: entity ROrgPrSimLib.mipsCtrlFsm(behavioral)
		port map(clk => clk,
						rst => rst,
						op => sInstruction(REG_WIDTH-1 downto 26),
						regDst => sregDst,
						regWrite => sregWrite,
						memRead => smemRead,
						memToReg => smemToReg,
						aluOp => saluOp,
						memWrite => smemWrite,
						aluSrcA => saluSrcA,
						aluSrcB => saluSrcB,
						pcSrc => spcSrc,
						irWrite => sirWrite,
						IorD => sIorD,
						pcWrite => spcWrite,
						pcWriteCond => spcWriteCond,
						mipsCtrlState_debug => mipsCtrlState_debug);

	-- Sign extend
	SIGN_EXTEND: entity ROrgPrSimLib.signExtend(behavioral)
		generic map(INPUT_WIDTH => 16,
								OUTPUT_WIDTH => REG_WIDTH)
		port map(number => sSignedNumber,
						signExtNumber => sSignExtendedNumber);

	-- SLL after sign extend
	SLL_SIGN_EXTENDED: entity ROrgPrSimLib.leftShifter(behavioral)
		generic map(WIDTH => REG_WIDTH,
								SHIFT_AMOUNT => 2)
		port map(number => sVectSignExtend,
						shiftedNumber => sShiftedNumber);

	swritedata <= smemDatareg when smemToReg = '1'
								else sALUout when smemToReg = '0'
								else (others => 'X');

	swriteregister <=	sInstruction(20 downto 16) when sregDst = '0'
										else sInstruction(15 downto 11) when sregDst = '1'
										else (others => 'X');

	saluA <=	sPC when saluSrcA = '0'
						else sA when saluSrcA = '1'
						else (others => 'X');

	with saluSrcB select
		saluB <=
			sB when "00",
			sSLVfour when "01",
			sVectSignExtend when "10",
			sShiftedNumber when "11",
			(others => 'X') when others;

	with spcSrc select
		stoPC <=
			sALUresult when "00",
			sALUout when "01",
			sJumpAddress when "10",
			(others => 'X') when others;

	memDataReg : entity ROrgPrSimLib.reg(behavioral)
		generic map(WIDTH => REG_WIDTH)
		port map(clk => clk,
						rst => '0',
						en => '1',
						D => smemData,
						Q => smemDatareg);

	ALUoutReg : entity ROrgPrSimLib.reg(behavioral)
		generic map(WIDTH => REG_WIDTH)
		port map(clk => clk,
						rst => '0',
						en => '1',
						D => sALUresult,
						Q => sALUout);

	AReg : entity ROrgPrSimLib.reg(behavioral)
		generic map(WIDTH => REG_WIDTH)
		port map(clk => clk,
						rst => '0',
						en => '1',
						D => sReaddata1,
						Q => sA);

	BReg : entity ROrgPrSimLib.reg(behavioral)
		generic map(WIDTH => REG_WIDTH)
		port map(clk => clk,
							rst => '0',
							en => '1',
							D => sReaddata2,
							Q => sB);

	INSTRUCTION_REG : entity ROrgPrSimLib.reg(behavioral)
		generic map(WIDTH => REG_WIDTH)
		port map(clk => clk,
							rst => '0',
							en => sirWrite,
							D => smemData,
							Q => sInstruction);

	-- 7 Segment Debug
	PC_7_SEG_DIGITS_DEBUG_0: entity ROrgPrSimLib.bin2Char(behavioral)
		port map(bin => sPC(3 downto 0),
		 					bitmask => pc7SegDigits_debug(0));

	PC_7_SEG_DIGITS_DEBUG_1: entity ROrgPrSimLib.bin2Char(behavioral)
		port map(bin => sPC(7 downto 4),
		 					bitmask => pc7SegDigits_debug(1));

	PC_7_SEG_DIGITS_DEBUG_2: entity ROrgPrSimLib.bin2Char(behavioral)
		port map(bin => sPC(11 downto 8),
		 					bitmask => pc7SegDigits_debug(2));

	PC_7_SEG_DIGITS_DEBUG_3: entity ROrgPrSimLib.bin2Char(behavioral)
		port map(bin => sPC(15 downto 12),
		 					bitmask => pc7SegDigits_debug(3));

end structural;
