library ieee;
use ieee.std_logic_1164.all;

entity mux2 is
	port( a, b: in  std_logic;
	      sel        : in  std_logic;
	      y          : out std_logic);
end mux2;

architecture behavioral of mux2 is
begin

	y <= a when sel = '0' else
			 b when sel = '1' else 'X';
end behavioral;
