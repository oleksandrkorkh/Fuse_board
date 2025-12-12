#pragma once

namespace board
{

struct BoardStatus
{
  float temperature; // Board temperature (C)
  float v_in;        // Voltage in (V)
};

void handle_board_data();
BoardStatus get_board_status();

void initialize_board_data();

}


