library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity leftShifter is
	generic(WIDTH : integer;
	        SHIFT_AMOUNT : integer);
	port(number : in std_logic_vector(WIDTH - 1 downto 0);
		 shiftedNumber : out std_logic_vector(WIDTH - 1 downto 0));
end leftShifter;

architecture behavioral of leftShifter is
begin

	process (number)
	variable nr : std_logic_vector(WIDTH - 1 downto 0);
	variable save1, save2 : std_logic;
	begin
		nr := number;
		for i in 0 to SHIFT_AMOUNT - 1 loop
			save1 := nr(0);
			save2 := nr(1);
			nr(0) := '0';
			for j in 1 to WIDTH - 2 loop
				nr(j) := save1;
				save1 := save2;
				save2 := nr(j+1);
			end loop;
			nr(WIDTH - 1) := save1;
		end loop;
		shiftedNumber <= nr;
	end process;
	
end behavioral;
