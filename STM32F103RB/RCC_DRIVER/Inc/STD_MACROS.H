/*
 * STD_MACROS.H
 *
 *  Created on: Nov 18, 2024
 *      Author: Sherif Abdelhay
 */

#ifndef STD_MACROS_H_
#define STD_MACROS_H_

#define SET_BIT(reg_name,bit)		reg_name |=(1<<bit)
#define CLR_BIT(reg_name,bit)		reg_name &=(~(1<<bit))
#define TOG_BIT(reg_name,bit)		reg_name ^=(1<<bit)
#define READ_BIT(reg_name,bit)		((reg_name&(1<<bit))>>bit)

#endif /* STD_MACROS_H_ */
