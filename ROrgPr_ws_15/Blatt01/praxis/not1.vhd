library ieee;
use ieee.std_logic_1164.all;

entity not1 is
	port(i : in std_logic;
		 o : out std_logic);
end not1;

architecture behavioral of not1 is
begin
	o <= not i;
end behavioral;
