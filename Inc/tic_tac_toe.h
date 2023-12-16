
#ifndef TIC_TAC_TOE_H
#define TIC_TAC_TOE_H

#define CELL_SIZE 60

#include "board_image.h"
#include "x_image.h"
#include "o_image.h"

// top left corner of the board
#define BOARD_X 30 
#define BOARD_Y 30

struct CellIndexes {
	int i;
	int j;
};

int TryGetCellTouched(int xPos, int yPos);
void DrawXInsideCell(int cellNumber);
void DrawCircleInCell(int cellNumber);
char CellIsempty(int cellNumber, int board[3][3]);

// gets the indexes of the pressed cell
struct CellIndexes GetCellIndexes(int cellNumber)
{
	struct CellIndexes ci;
	
	int currentCell = 1;
	for (int i = 0; i < 3; i++) 
	{
    for (int j = 0; j < 3; j++) 
		{
				if (currentCell == cellNumber)
				{
					ci.i = i;
					ci.j = j;
					
					return ci;
				}
				
				currentCell++;
    }
  }
	
	return ci;
}

// Returns 1 if the cell is empty, 0 otherwise
char CellIsEmpty(int cellNumber, int board[3][3], struct CellIndexes ci)
{
		if (board[ci.i][ci.j] == -1)
			return 1;
		else
			return 0;
}

/*
	Given the rects X, Y top left corner, and the width and heigth of the rect, returns 1 if the 
	position is within this rect
*/
int isInsideRect(int xPos, int yPos, int rectX, int rectY, int rectWidth, int rectHeigth)
{
		if (xPos > rectX 
				&& yPos > rectY 
				&& xPos <= (rectX + rectWidth)
				&& yPos <= (rectY + rectHeigth))
		{
			return 1;
		}
		
		return 0;
}

/*
	Returns a number from 1 to 9 for what cell in the game board was touched
	Returns -1 if no cell was touched
*/
int TryGetCellTouched(int xPos, int yPos)
{
  int cellNumber = 1;
	
  for (int i = 0; i < 3; i++) 
	{
    for (int j = 0; j < 3; j++) 
		{
			
      if (isInsideRect(
						xPos, 
						yPos, 
						BOARD_X + j * CELL_SIZE, 
						BOARD_Y + i * CELL_SIZE,
						CELL_SIZE, CELL_SIZE)) 
			{
        return cellNumber;
      }

      cellNumber++;
    }
  }

  return -1;
}

/*
	Draws an X inside one of the boards cells (1-9)
*/
void DrawXInsideCell(int cellNumber)
{
	int rectX;
	int rectY;
	
	switch(cellNumber)
	{
		case 1:
			rectX = BOARD_X;
			rectY = BOARD_Y;
			break;
		
		case 2:
			rectX = BOARD_X + CELL_SIZE;
			rectY = BOARD_Y;
			break;
		
		
		case 3:
			rectX = BOARD_X + CELL_SIZE * 2;
			rectY = BOARD_Y;
			break;
		
		case 4:
			rectX = BOARD_X;
			rectY = BOARD_Y + CELL_SIZE;
			break;
		
		case 5:
			rectX = BOARD_X + CELL_SIZE;
			rectY = BOARD_Y + CELL_SIZE;
			break;
		
		case 6:
			rectX = BOARD_X + CELL_SIZE * 2;
			rectY = BOARD_Y + CELL_SIZE;
			break;
		
		case 7:
			rectX = BOARD_X;
			rectY = BOARD_Y + CELL_SIZE * 2;
			break;
		
		case 8:
			rectX = BOARD_X + CELL_SIZE;
			rectY = BOARD_Y + CELL_SIZE * 2;
			break;
		
		case 9:
			rectX = BOARD_X + CELL_SIZE * 2;
			rectY = BOARD_Y + CELL_SIZE * 2;
			break;
	}
	
		DrawXAtPosition(rectX, rectY);
//	int line1X1 = rectX + 3;
//	int line1Y1 = rectY + 3;
//	int line1X2 = rectX + CELL_SIZE - 3;
//	int line1Y2 = rectY + CELL_SIZE - 3;
//	
//	int line2X1 = rectX + CELL_SIZE - 3;
//	int line2Y1 = rectY + 3;
//	int line2X2 = rectX + 3;
//	int line2Y2 = rectY + CELL_SIZE - 3;
//	
//	BSP_LCD_DrawLine(line1X1, line1Y1, line1X2, line1Y2);
//	BSP_LCD_DrawLine(line2X1, line2Y1, line2X2, line2Y2);
}

/*
	Draws a circle inside a cell.
*/
void DrawCircleInCell(int cellNumber)
{
	int currentCell = 1;
	
  for (int i = 0; i < 3; i++) 
	{
    for (int j = 0; j < 3; j++) 
		{
			if (currentCell == cellNumber)
			{
				int xPos = BOARD_X + j * CELL_SIZE;
				int yPos = BOARD_Y + i * CELL_SIZE; 
				
				Draw0AtPosition(xPos, yPos);
				//BSP_LCD_DrawCircle(xPos, yPos, CELL_SIZE / 2 - 5);
				return;
			}

      currentCell++;
    }
  }
}

#endif
