/*****************************************************************************/
/*                                                                           */
/*     Project: ESE Project - "SortBot" implemented with E/S-Code            */
/*   Copyright: Copyright © 2005. All Rights Reserved                        */
/*    Compiler: Microsoft Visual C++ .NET - Target Win32                     */
/*     Company: Dept of ComSci, University of Salzburg                       */
/*     Authors: Rainer Trummer <rtrummer@cosy.sbg.ac.at>                     */
/*              Robert Loeffelberger <rolo@aon.at>                           */
/*        Date: January 5, 2005                                              */
/*                                                                           */
/*****************************************************************************/

#define SORTBOT_TERMINAL // enables LegOS code generation

#ifdef SORTBOT_TERMINAL
  #include <dsensor.h>
  #include <dmotor.h>
  #include <unistd.h>
  #include <conio.h>
#else
  #include <windows.h>
  #include <stdlib.h>
  #include <stdio.h>
  #include <time.h>
#endif


/*****************************************************************************/
/*                                                                           */
/*  Definitions                                                              */
/*                                                                           */
/*****************************************************************************/

// E/S-Code Instruction Formats
//
// release(t):    |31  4-bit code  28|  12-bit deadline   16|  16-bit task ID           0|
// dispatch(t):   |31  4-bit code  28|  12-bit deadline   16|  16-bit task ID           0|
// future(n,a):   |31  4-bit code  28|  12-bit delay      16|  16-bit relative address  0|
// if(c,a):       |31  4-bit code  28|  12-bit condition  16|  16-bit relative address  0|
// jump(a):       |31  4-bit code  28|              28-bit absolute address             0|
// idle(n):       |31  4-bit code  28|              28-bit predefined delay             0|
// return:        |31  4-bit code  28|                    (not used)                    0|
//
#define ABS_ADDR       0x0fffffffUL  // 00001111111111111111111111111111
#define REL_ADDR       0x0000ffffUL  // 00000000000000001111111111111111
#define POS_CODE       28            // instruction >> 28
#define POS_INFO       16            // instruction >> 16

// E/S-Code Instruction Set
//
#define INST_RELEASE   0x01  // release(t): insert task t into task set
#define INST_DISPATCH  0x02  // dispatch(t): execute task t until t completes
#define INST_FUTURE    0x03  // future(n,a): wait n time instants before jumping to address a
#define INST_IF        0x04  // if(c,a): branch to address a if condition c evaluates to true
#define INST_JUMP      0x05  // jump(a): jump to absolute address a
#define INST_IDLE      0x06  // idle(t): wait n time instants until E-machine wakes up
#define INST_RETURN    0x07  // return: return from E-code interpreter

// Global Constants
//
#define NUM_TASKS      2     // number available task functions
#define BASE_FREQ      5     // base frequency of E-machine

// Type Definitions
//
typedef unsigned long  word; // always 32 bits
#ifndef SORTBOT_TERMINAL
typedef enum { off, fwd, rev };
#endif


/*****************************************************************************/
/*                                                                           */
/*  Macros                                                                   */
/*                                                                           */
/*****************************************************************************/

#ifndef SORTBOT_TERMINAL
#define randomize( ) srand( (unsigned int) time( NULL ) )
#define random( __n ) (word)(rand( ) / (RAND_MAX + 1.0) * (__n))
#endif


/*****************************************************************************/
/*                                                                           */
/*  E-Code Block                                                             */
/*                                                                           */
/*****************************************************************************/

const word E_CODE[] =
{
    0x40000005UL, // a00: if( halted, a05 )           {offset = 5 instructions}
    0x40010007UL, // a01: if( is_item, a08 )          {offset = 7 instructions}
    0x40020009UL, // a02: if( stop_c, a11 )           {offset = 9 instructions}
    0x30050000UL, // a03: future( 5, e_code )         {future = base frequency}
    0x70000000UL, // a04: return

    0x10050000UL, // a05: release( halt_program[5] )  {dispatch task after 5 ms}
    0x30050000UL, // a06: future( 5, e_code )         {future = base frequency}
    0x70000000UL, // a07: return

    0x11f40001UL, // a08: release( set_motor_c[500] ) {dispatch task after 500 ms}
    0x30050000UL, // a09: future( 5, e_code )         {future = base frequency}
    0x70000000UL, // a10: return

    0x10050001UL, // a11: release( set_motor_c[5] )   {dispatch task after 5 ms}
    0x30050000UL, // a12: future( 5, e_code )         {future = base frequency}
    0x70000000UL  // a13: return
};


/*****************************************************************************/
/*                                                                           */
/*  S-Code Block                                                             */
/*                                                                           */
/*****************************************************************************/

const word S_CODE[] =
{
    0x20000000UL, // a00: dispatch( halt_program[0] ) {execute task if delay = 0}
    0x20000001UL, // a01: dispatch( set_motor_c[0] )  {execute task if delay = 0}
    0x50000001UL, // a02: jump( s_code )              {set next S-code address}
    0x70000000UL  // a03: return
};


/*****************************************************************************/
/*                                                                           */
/*  Global Variables                                                         */
/*                                                                           */
/*****************************************************************************/

static int  MotorDir;
static bool HoldData;
static word DelayCnt;
static word TaskSet[NUM_TASKS];


/*****************************************************************************/
/*                                                                           */
/*  Initialization/Deinitialization                                          */
/*                                                                           */
/*****************************************************************************/

#ifdef SORTBOT_TERMINAL

// Initialize all components
//
void initialize( void )
{
    ds_active( &SENSOR_2 );
    ds_active( &SENSOR_3 );
    motor_a_speed( MAX_SPEED );
    motor_c_speed( MAX_SPEED / 2 );
    motor_a_dir( fwd );
}

// Deinitialize all components
//
void deinitialize( void )
{
    motor_a_speed( off );
    motor_c_speed( off );
    ds_passive( &SENSOR_2 );
    ds_passive( &SENSOR_3 );
}

#endif


/*****************************************************************************/
/*                                                                           */
/*  Condition Functions                                                      */
/*                                                                           */
/*****************************************************************************/

// Returns TRUE if user presses [halt] (highest priority)
//
bool halted( void )
{
#ifdef SORTBOT_TERMINAL
    return( TOUCH_2 );
#else
    return( !!(GetKeyState( VK_RETURN ) & 0x8000) );
#endif
}

// Returns TRUE if sensor detects an item
//
bool is_item( void )
{
#ifndef SORTBOT_TERMINAL
    int LIGHT_3 = random( 17 ) + 27;
#endif

    if( !HoldData )
    {
        if( LIGHT_3 < 30 ) // black item --> move right
        {
            MotorDir = fwd;
            HoldData = true;
            return( true );
        }

        if( LIGHT_3 > 40 ) // yellow item --> move left
        {
            MotorDir = rev;
            HoldData = true;
            return( true );
        }
    }

    return( false );
}

// Returns TRUE if motor C can be stopped
//
bool stop_c( void )
{
    if( !HoldData )
    {
#ifdef SORTBOT_TERMINAL
        if( ++DelayCnt > 250 )
#else
        if( ++DelayCnt > 5 )
#endif
        {
            MotorDir = off;
            HoldData = true;
            return( true );
        }
    }

    return( false );
}


/*****************************************************************************/
/*                                                                           */
/*  Task Functions                                                           */
/*                                                                           */
/*****************************************************************************/

// Immediately halt program (highest priority)
//
void halt_program( void )
{
#ifdef SORTBOT_TERMINAL
    deinitialize( );
#endif
    exit( 0 );
}

// Set direction of motor C
//
void set_motor_c( void )
{
#ifdef SORTBOT_TERMINAL
    motor_c_dir( (MotorDirection) MotorDir );
#else
    if( MotorDir == fwd )
    {
        printf( "-->\n" );
    }
    else
    if( MotorDir == rev )
    {
        printf( "<--\n" );
    }
    else
    {
        printf( "off\n" );
    }
#endif
    HoldData = false;
    DelayCnt = 0;
}


/*****************************************************************************/
/*                                                                           */
/*  Address Tables                                                           */
/*                                                                           */
/*****************************************************************************/

// Table that references E/S-code blocks
//
const word *CodeRef[] = { E_CODE, S_CODE };

// Table that references conditions
//
bool (*CondRef[])( void ) = { halted, is_item, stop_c };

// Table that references task functions
//
void (*TaskRef[])( void ) = { halt_program, set_motor_c };


/*****************************************************************************/
/*                                                                           */
/*  E/S-Code Interpreter                                                     */
/*                                                                           */
/*****************************************************************************/

const word *interpret( const word *code )
{
#ifndef SORTBOT_TERMINAL
    static double d;
#endif

	const  word *pc = code;
    static word op, info, addr;

    for( ; ; ++pc )
    {
		op = *pc >> POS_CODE;

        if( op == INST_RELEASE )
		{
            if( !TaskSet[addr = *pc & REL_ADDR] )
            {
                TaskSet[addr] = *pc;
            }
		}
        else
        if( op == INST_DISPATCH )
		{
            if( TaskSet[addr = *pc & REL_ADDR] )
            {
                if( (info = ((TaskSet[addr] & ABS_ADDR) >> POS_INFO) - BASE_FREQ) )
                {
                    TaskSet[addr] = (INST_RELEASE << POS_CODE) | (info << POS_INFO) | addr;
                }
                else
                {
                    TaskRef[addr]( );
                    TaskSet[addr] = 0;
                }
            }
		}
        else
        if( op == INST_FUTURE )
		{
            code = CodeRef[*pc & REL_ADDR];
            info = (*pc & ABS_ADDR) >> POS_INFO;
#ifdef SORTBOT_TERMINAL
            msleep( info );
#else
            for( d = info; d > 0.0; d -= 5e-6 )
                ;
#endif
		}
        else
        if( op == INST_IF )
		{
            if( CondRef[(*pc & ABS_ADDR) >> POS_INFO]( ) )
            {
                pc += (*pc & REL_ADDR) - 1;
            }
		}
        else
        if( op == INST_JUMP )
		{
            code = CodeRef[*pc & ABS_ADDR];
		}
        else
        if( op == INST_IDLE )
		{
            info = *pc & ABS_ADDR;
#ifdef SORTBOT_TERMINAL
            msleep( info );
#else
            for( d = info; d > 0.0; d -= 5e-6 )
                ;
#endif
		}
        else
        if( op == INST_RETURN )
		{
            return( code );
		}
        else
        {
#ifdef SORTBOT_TERMINAL
            cputs( "Err" );
#else
            printf( "Unknown instruction!\n" );
#endif
            return( 0 );
        }
    }
}


/*****************************************************************************/
/*                                                                           */
/*  OS Kernel                                                                */
/*                                                                           */
/*****************************************************************************/

int main( void )
{
    // Set E/S-code pointers to first addresses
    //
    const word *e_code_ptr = CodeRef[0];
    const word *s_code_ptr = CodeRef[1];

    MotorDir = off;
    HoldData = false;
    DelayCnt = 0;

#ifdef SORTBOT_TERMINAL
    initialize( );
#else
    randomize( );
    printf( "\nPress [Enter] to halt SortBot\n\n" );
#endif

    for( ; ; )
    {
        // Execute E-code block
        //
        if( (e_code_ptr = interpret( e_code_ptr )) == NULL )
        {
            break;
        }

        // Execute S-code block
        //
        if( (s_code_ptr = interpret( s_code_ptr )) == NULL )
        {
            break;
        }
    }

#ifdef SORTBOT_TERMINAL
    deinitialize( );
#endif

    return( 1 );
}

// End of file.


