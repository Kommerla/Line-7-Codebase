/*
 * load_cell.h
 *
 *  Created on: 15-Jul-2026
 *      Author: ESE
 */

#ifndef INC_LOAD_CELL_H_
#define INC_LOAD_CELL_H_


static void LoadCellComputeCRC(uint8_t *buf, uint16_t len);
bool LoadCellReadTest(LoadCellHandle *loadcell);
#endif /* INC_LOAD_CELL_H_ */
