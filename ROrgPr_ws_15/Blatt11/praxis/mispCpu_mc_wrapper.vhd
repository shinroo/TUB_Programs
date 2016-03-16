library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

library work;
use work.proc_config.all;

entity mispCpu_mc_wrapper is
    Port ( clk : in STD_LOGIC;
           seg : out STD_LOGIC_VECTOR (6 downto 0);
           an : out STD_LOGIC_VECTOR (3 downto 0);
           led : out STD_LOGIC_VECTOR (9 downto 0);
           btnC : in STD_LOGIC;
           btnD : in STD_LOGIC);
end mispCpu_mc_wrapper;

architecture Behavioral of mispCpu_mc_wrapper is

signal cpuClk, cpuRst : std_logic;
signal curPc : pc_7seg_digits_type;
signal curState : mips_ctrl_state_type;

signal btnPressed : std_logic;
signal counter : unsigned(23 downto 0) := (others => '0');

begin
    cpu: entity work.mipsCpu_mc
    generic map(
        INIT_FILE_NAME => "clip.mif"
    )
    port map( 
        clk => cpuClk,
        rst => cpuRst,
        ramElements_debug => open,
        registers_debug => open,
        mipsCtrlState_debug => curState,
        pc7SegDigits_debug => curPC
    );

    --LEDs
    led(0) <= '0' when curState = INSTR_FETCH else '1';
	led(1) <= '0' when curState = INSTR_DECODE else '1';
	led(2) <= '0' when curState = BRANCH_COMPL else '1';
	led(3) <= '0' when curState = JUMP_COMPL else '1';
	led(4) <= '0' when curState = EXECUTION else '1';
	led(5) <= '0' when curState = RTYPE_COMPL else '1';
	led(6) <= '0' when curState = MEM_ADDR_CALC else '1';
	led(7) <= '0' when curState = MEM_WRITE else '1';
	led(8) <= '0' when curState = MEM_READ else '1';
	led(9) <= '0' when curState = MEM_READ_COMPL else '1';
	
    -- 7Seg
	cnt: process(clk)
	begin
	   if rising_edge(clk) then
	       counter <= counter + "1";
	   end if;
	end process;
	
	an(0) <= '0' when counter(16 downto 15) = "00" else '1';
	an(1) <= '0' when counter(16 downto 15) = "01" else '1';
    an(2) <= '0' when counter(16 downto 15) = "10" else '1';
    an(3) <= '0' when counter(16 downto 15) = "11" else '1';
    
    seg <= curPc(to_integer(unsigned(counter(16 downto 15))));
	
	-- Forward Handling
	fwd: process(clk)
	begin
        if rising_edge(clk) then
           cpuClk <= cpuClk;
           btnPressed <= btnPressed;
           counter <= counter + "1";
	       if counter = (23 downto 0 => '0') then
               cpuRst <= btnD;
	           if btnC = '0' then
	               btnPressed <= '0';
	           else
	               if btnPressed = '1' then
	                   cpuClk <= '0';
	               else
	                   cpuClk <= '1';
	               end if;
	               btnPressed <= '1';
	           end if;
	       end if;
	   end if;
	end process;
end Behavioral;