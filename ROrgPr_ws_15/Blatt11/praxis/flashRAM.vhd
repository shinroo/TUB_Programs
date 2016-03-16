library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library std;
use std.standard.all;
use std.textio.all;

library work;
use work.proc_config.all;

entity flashRAM is
	generic(NUM_ELEMENTS : integer;
			LOG2_NUM_ELEMENTS : integer;
			ELEMENT_WIDTH : integer;
			INIT_FILE_NAME : string);
	port(clk : in std_logic;
		 address : in std_logic_vector(LOG2_NUM_ELEMENTS - 1 downto 0);
		 writeEn : in std_logic;
		 writeData : in std_logic_vector(ELEMENT_WIDTH - 1 downto 0);
		 readEn : in std_logic;
		 readData : out std_logic_vector(ELEMENT_WIDTH - 1 downto 0);
		 ramElements_debug : out ram_elements_type);
end flashRAM;

architecture behavioral of flashRAM is
    impure function InitRamFromFile (RamFileName : in string) return ram_elements_type is
        FILE RamFile : text is in RamFileName;
        variable RamFileLine : line;
        variable dataBuffer : bit_vector(ELEMENT_WIDTH - 1 downto 0);
        variable RAM : ram_elements_type;
    begin
        for i in ram_elements_type'range loop
            readline(RamFile, RamFileLine);
            read(RamFileLine, dataBuffer);
            RAM(i) := to_stdlogicvector(dataBuffer);
        end loop;
        return RAM;
    end function;

signal ramElements : ram_elements_type := InitRamFromFile(INIT_FILE_NAME);

begin

	process(clk)
	
	begin
	
		if rising_edge(clk) then
			-- write
			if writeEn = '1' then
				ramElements(to_integer(unsigned(address))) <= writeData;
			end if;

			-- read
			if readEn = '1' then
				if writeEn = '1' then
					readData <= writeData;
				else
					readData <= ramElements(to_integer(unsigned(address)));
				end if;
			end if;
		end if;
        
        -- synthesis translate_off
        ramElements_debug <= ramElements;
        -- synthesis translate_on

	end process;

end behavioral;