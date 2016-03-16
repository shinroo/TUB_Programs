library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library ROrgPrSimLib;
use ROrgPrSimLib.proc_config.all;

entity regFile is
	generic(NUM_REGS : integer;
			LOG2_NUM_REGS : integer;
			REG_WIDTH : integer);
	port(clk : in std_logic;
		 rst : in std_logic;
		 readAddr1 : in std_logic_vector(LOG2_NUM_REGS - 1 downto 0);
		 readData1 : out std_logic_vector(REG_WIDTH - 1 downto 0);
		 readAddr2 : in std_logic_vector(LOG2_NUM_REGS - 1 downto 0);
		 readData2 : out std_logic_vector(REG_WIDTH - 1 downto 0);
		 writeEn : in std_logic;
		 writeAddr : in std_logic_vector(LOG2_NUM_REGS - 1 downto 0);
		 writeData : in std_logic_vector(REG_WIDTH - 1 downto 0);
		 reg_vect_debug : out reg_vector_type);
end regFile;

architecture structural of regFile is

component addrDecoder is
	generic(ADDR_WIDTH : integer;
			POW2_ADDR_WIDTH : integer);
	port(address : in std_logic_vector(ADDR_WIDTH - 1 downto 0);
		 bitmask : out std_logic_vector(POW2_ADDR_WIDTH - 1 downto 0));
end component addrDecoder;

component reg is
	generic(
		WIDTH : integer);
	port(
		clk : in  std_logic;
		rst : in  std_logic;
		en  : in  std_logic;
		D   : in  std_logic_vector( WIDTH - 1 downto 0 );
		Q   : out std_logic_vector( WIDTH - 1 downto 0 ));
end component reg;	
	signal reg_vect : reg_vector_type;
	signal bitmask, read1, read2, writeArray : std_logic_vector( NUM_REGS - 1 downto 0);
begin

bitmaske:	entity work.addrDecoder (behavioral)
			generic map (ADDR_WIDTH => LOG2_NUM_REGS,
				    POW2_ADDR_WIDTH => NUM_REGS)
			port map (address => writeAddr,
				bitmask => bitmask);

read_1:		entity work.addrDecoder (behavioral)
			generic map (ADDR_WIDTH => LOG2_NUM_REGS,
				    POW2_ADDR_WIDTH => NUM_REGS)
			port map (address => readAddr1,
				bitmask => read1);

read_2:		entity work.addrDecoder (behavioral)
			generic map (ADDR_WIDTH => LOG2_NUM_REGS,
				    POW2_ADDR_WIDTH => NUM_REGS)
			port map (address => readAddr2,
				bitmask => read2);

writeArr:	for i in reg_vect'range generate
	 		writeArray(i) <= '1' when (bitmask(i) and writeEn) = '1' else '0';
		end generate;

registers:		for i in reg_vect'range generate
register_i:			entity work.reg (behavioral)		
				generic map( WIDTH => REG_WIDTH)
				port map( clk => clk,
					rst => rst,
					en  => writeArray(i),
					D => writeData,
					Q => reg_vect(i));
		end generate;
	
readData:	for i in reg_vect'range generate
			readData1 <= reg_vect(i) when read1(i) = '1' else (others => 'L');
			readData2 <= reg_vect(i) when read2(i) = '1' else (others => 'L');	
		end generate;
	-- Inhalt des Registerspeichers über den Debug-Port nach außen führen
	reg_vect_debug <= reg_vect;

end structural;
