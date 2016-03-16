library ieee;
use ieee.std_logic_1164.all;

library ROrgPrSimLib;
use ROrgPrSimLib.proc_config.mips_ctrl_state_type;
use ROrgPrSimLib.mipsISA.all;

entity mipsCtrlFsm is
port(clk : in std_logic;
	 rst : in std_logic;
	 op : in std_logic_vector(5 downto 0);
	 regDst : out std_logic;
	 memRead : out std_logic;
	 memToReg : out std_logic;
	 aluOp : out std_logic_vector(1 downto 0);
	 memWrite : out std_logic;
	 regWrite : out std_logic;
	 aluSrcA : out std_logic;
	 aluSrcB : out std_logic_vector(1 downto 0);
	 pcSrc : out std_logic_vector(1 downto 0);
	 irWrite : out std_logic;
	 IorD : out std_logic;
	 pcWrite : out std_logic;
	 pcWriteCond : out std_logic;

	 -- debug port
	 mipsCtrlState_debug : out mips_ctrl_state_type);
end mipsCtrlFsm;

architecture behavioral of mipsCtrlFsm is

	signal state_reg : mips_ctrl_state_type;

begin

	-- Beschreibung der MIPS Control FSM hier einfügen

	--constant R_FORMAT_OPCODE : std_logic_vector(5 downto 0) := "000000";
	--constant ADD_FUNCT : std_logic_vector(5 downto 0) := "100000";
	--constant SUB_FUNCT : std_logic_vector(5 downto 0) := "100010";
	--constant AND_FUNCT : std_logic_vector(5 downto 0) := "100100";
	--constant  OR_FUNCT : std_logic_vector(5 downto 0) := "100101";
	--constant SLT_FUNCT : std_logic_vector(5 downto 0) := "101010";

	--constant LW_OPCODE : std_logic_vector(5 downto 0) := "100011";
	--constant SW_OPCODE : std_logic_vector(5 downto 0) := "101011";
	--constant BEQ_OPCODE : std_logic_vector(5 downto 0) := "000100";
	--constant J_OPCODE : std_logic_vector(5 downto 0) := "000010";

	state_process: process(clk, rst)
	begin
			--insert state process here
			if rst = '1' then state_reg <= INSTR_FETCH;
			elsif rising_edge(clk) then
					case state_reg is
							--0 Instruction Fetch
							when INSTR_FETCH => state_reg <= INSTR_DECODE;
							--1 Instruction decode/register fetch
							when INSTR_DECODE =>
									if op = LW_OPCODE or op = SW_OPCODE then state_reg <= MEM_ADDR_CALC;
								  elsif op = R_FORMAT_OPCODE then state_reg <= EXECUTION;
									elsif op = BEQ_OPCODE then state_reg <= BRANCH_COMPL;
									elsif op = J_OPCODE then state_reg <= JUMP_COMPL;
								  else state_reg <= INSTR_FETCH;
									end if;
							--2 Memory address computation
							when MEM_ADDR_CALC =>
									if op = LW_OPCODE then state_reg <= MEM_READ;
									elsif op = SW_OPCODE then state_reg <= MEM_WRITE;
									else state_reg <= INSTR_FETCH;
									end if;
							--3 Memory access
							when MEM_READ => state_reg <= MEM_READ_COMPL;
							--4 Memory read completion step
							when MEM_READ_COMPL => state_reg <= INSTR_FETCH;
							--5 Memory access
							when MEM_WRITE => state_reg <= INSTR_FETCH;
							--6 Execution
							when EXECUTION => state_reg <= RTYPE_COMPL;
							--7 R-type completion
							when RTYPE_COMPL => state_reg <= INSTR_FETCH;
							--8 Branch completion
							when BRANCH_COMPL => state_reg <= INSTR_FETCH;
							--9 Jump completion
							when JUMP_COMPL => state_reg <= INSTR_FETCH;
				end case;
		end if;
	end process;

	output_process: process(state_reg)
	begin
			--insert output process here
			case state_reg is
					--0 Instruction Fetch
					when INSTR_FETCH =>
							regDst <= '0';
							regWrite <= '0';
							ALUSrcA <= '0';
							memRead <= '1';
							memWrite <= '0';
							memToReg <= '0';
							IorD <= '0';
							irWrite <= '1';
							pcWrite <= '1';
							pcWriteCond <= '0';
							ALUOp <= "00";
							ALUSrcB <= "01";
							pcSrc <= "00";
					--1 Instruction decode/register fetch
					when INSTR_DECODE =>
							memRead <= '0';
							irWrite <= '0';
							pcWrite <= '0';
							ALUSrcB <= "11";
					--2 Memory address computation
					when MEM_ADDR_CALC =>
							ALUSrcA <= '1';
							ALUSrcB <= "10";
					--3 Memory access
					when MEM_READ =>
							ALUSrcA <= '0';
							memRead <= '1';
							IorD <= '1';
							ALUSrcB <= "00";
					--4 Memory read completion step
					when MEM_READ_COMPL =>
							regWrite <= '1';
							memRead <= '0';
							memtoReg <= '1';
							IorD <= '0';
					--5 Memory access
					when MEM_WRITE =>
							ALUSrcA <= '0';
							memWrite <= '1';
							IorD <= '1';
							ALUSrcB <= "00";
					--6 Execution
					when EXECUTION =>
							ALUSrcA <= '1';
							ALUOp <= "10";
							ALUSrcB <= "00";
					--7 R-type completion
					when RTYPE_COMPL =>
							regDst <= '1';
							regWrite <= '1';
							ALUSrcA <= '0';
							ALUOp <= "00";
					--8 Branch completion
					when BRANCH_COMPL =>
							ALUSrcA <= '1';
							pcWriteCond <= '1';
							ALUOp <= "01";
							ALUSrcB <= "00";
							pcSrc <= "01";
					--9 Jump completion
					when JUMP_COMPL =>
							pcWrite <= '1';
							ALUSrcB <= "00";
							pcSrc <= "10";
		end case;
	end process;

	mipsCtrlState_debug <= state_reg;

end behavioral;
