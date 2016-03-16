library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library ROrgPrSimLib;
use ROrgPrSimLib.proc_config.all;

entity ram is
	generic(
		NUM_ELEMENTS      : integer;
		LOG2_NUM_ELEMENTS : integer;
		ELEMENT_WIDTH     : integer);
	port(
		clk               : in  std_logic;
		address           : in  std_logic_vector( LOG2_NUM_ELEMENTS - 1 downto 0 );
		writeEn           : in  std_logic;
		writeData         : in  std_logic_vector( ELEMENT_WIDTH - 1 downto 0 );
		readEn            : in  std_logic;
		readData          : out std_logic_vector( ELEMENT_WIDTH - 1 downto 0 );
		ramElements_debug : out ram_elements_type);
end ram;

architecture behavioral of ram is

	-- Array, das alle Elemente des RAMs enthält
	signal ramElements : ram_elements_type := (others => (others => '0'));
	--signal readAdd : std_logic_vector ( LOG2_NUM_ELEMENTS - 1 downto 0);

begin

	-- RAM-Beschreibung hier einfügen
	Ramprocess: process(clk) is
	begin

		if(rising_edge(clk)) then --When clock = 1
			--WRITE
			if writeEn = '1' then
				ramElements(To_integer(unsigned(address))) <= writeData;
			end if;
			--READ
			if readEn = '1' then
				--readAdd <= address;
				readData <= ramElements(to_integer(unsigned(address)));
			end if;
		end if;

	end process Ramprocess;
	
	--readData <= ramElements(To_integer(unsigned(readAdd)));

	-- RAM-Inhalt über den Debug-Port nach außen führen
	ramElements_debug <= ramElements;

end behavioral;
