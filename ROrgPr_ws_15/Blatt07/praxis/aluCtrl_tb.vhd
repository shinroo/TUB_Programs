library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_textio.all;

library std;
use std.textio.all;

library ROrgPrSimLib;
use ROrgPrSimLib.proc_config.all;

entity aluCtrl_tb is
end aluCtrl_tb;


architecture behavioral of aluCtrl_tb is
        constant NUM_TESTS : integer := 4;
        constant ALUOp_WIDTH : integer := 2;
	constant f_WIDTH : integer := 6;
	constant operation_WIDTH : integer := 4;

	type aluOp_type is array (0 to NUM_TESTS - 1) of std_logic_vector(ALUOp_WIDTH - 1 downto 0);
	type f_type is array (0 to NUM_TESTS - 1) of std_logic_vector(f_WIDTH - 1 downto 0);
	type operation_type is array (0 to NUM_TESTS - 1) of std_logic_vector(operation_WIDTH - 1 downto 0);

	signal aluOp : aluOp_type := ("11", "01", "10", "00");
	signal f : f_type := ("XX1111", "XX1011", "XX1001", "XX0000");
	signal operation_ref : operation_type := ("0101", "0110", "0011", "0010");
	signal operation : operation_type;

begin


gen_4_Faelle: 	for i in 0 to 3 generate
aluCtrl_Instanziierung: entity work.aluCtrl(behavioral)
			port map(aluOp => aluOp(i), f => f(i),
				operation => operation(i));
		end generate;

TESTBENCH: process
		variable points : integer := 0;
		variable numErrors : integer := 0;
		variable l : line;
	begin
		for i in operation_ref'range loop
			wait for 5 ns;
			if (operation(i) /= operation_ref(i)) then
				write(l, time'image(now))
				write(l, string'(": Falsche Ausgabe(" & integer'image(i+1) &"): |"));
				write(l, operation(i));
				write(l, string'("|; erwartet: |"));
				write(l, operation_ref(i);
				write(l, string'("| "));
				writeline(OUTPUT, l);
				numErrors := numErrors + 1;
			end if;
		end loop;
		if (numErrors < 1) then
			report "Dein aluCtrl funktioniert einwandfrei! (aluCtrl: 2/2 Punkte)" severity note;
		elsif (numErrors = 1) then
			report "Dein aluCtrl funktioniert nicht einwandfrei! (aluCtrl: 1/2 Punkte)" severity note;
		elsif (numErrors > 1) then
			report "Dein aluCtrl funktioniert nicht einwandfrei! (aluCtrl: 0/2 Punkte)" severity error;
		end if;
		wait;
	end process TESTBENCH;

end behavioral;
