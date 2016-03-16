library ieee;
use ieee.std_logic_1164.all;

entity mux4 is
	port( a, b, c, d : in  std_logic;
	      sel        : in  std_logic_vector(1 downto 0);
	      y          : out std_logic);
end mux4;

architecture behavioral of mux4 is
begin
	y <= a when sel = "00" else
			 b when sel = "01" else
			 c when sel = "10" else
			 d when sel = "11" else 'X';

end behavioral;
