library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity addrDecoder is
	generic(ADDR_WIDTH : integer;
			POW2_ADDR_WIDTH : integer);
	port(address : in std_logic_vector(ADDR_WIDTH - 1 downto 0);
		 bitmask : out std_logic_vector(POW2_ADDR_WIDTH - 1 downto 0));
end addrDecoder;

architecture behavioral of addrDecoder is
begin

	-- Beschreibung des Adressdekoders hier einfügen
	gen: for i in 0 to POW2_ADDR_WIDTH-1 generate
	 bitmask(i) <= '1' when i=to_integer(unsigned(address)) else '0';
	end generate;
end behavioral;
