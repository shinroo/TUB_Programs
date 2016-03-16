library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library ROrgPrSimLib;
use ROrgPrSimLib.neq4;

entity neq4_tb is
end neq4_tb;

architecture behavioral of neq4_tb is

	signal a, b : std_logic_vector(3 downto 0);
	signal y, y_ref : std_logic;
	constant NUM_TESTCASES : integer := 256;

begin

	NEQ4_INST: entity work.neq4(logic)
	port map(a => a, b => b, y => y);

	NEQ4_INST_REF: entity ROrgPrSimLib.neq4(logic)
	port map(a => a, b => b, y => y_ref);

	process
		variable numErrors : integer := 0;
		variable tmp_i : std_logic_vector(7 downto 0);
	begin

		for i in 0 to NUM_TESTCASES - 1 loop
			
			tmp_i := std_logic_vector(to_unsigned(i, 8));
			a <= tmp_i(7 downto 4);
			b <= tmp_i(3 downto 0);

			wait for 5 ns;

			if y /= y_ref then
			    report "falsches Ergebnis am neq4-Modul (a = """ &
			    integer'image(to_integer(unsigned'("" & a(3)))) &
		        integer'image(to_integer(unsigned'("" & a(2)))) &
		        integer'image(to_integer(unsigned'("" & a(1)))) &
		        integer'image(to_integer(unsigned'("" & a(0)))) &
		        """, b = """ &
		        integer'image(to_integer(unsigned'("" & b(3)))) &
		        integer'image(to_integer(unsigned'("" & b(2)))) &
		        integer'image(to_integer(unsigned'("" & b(1)))) &
		        integer'image(to_integer(unsigned'("" & b(0)))) &
			    """): " & std_logic'image(y) & " (erwartet: " & std_logic'image(y_ref) & ")" severity error;
				numErrors := numErrors + 1;
			end if;

			wait for 5 ns;

		end loop;

		if numErrors = 0 then
			report "Das neq4-Modul funktioniert einwandfrei!" severity note;
		else
			report "Das neq4-Modul funktioniert nicht einwandfrei! (" & integer'image(numErrors) & " von " & integer'image(NUM_TESTCASES) & " Tests fehlerhaft)" severity failure;
		end if;

		wait;

	end process;	

end behavioral;
