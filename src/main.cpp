#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#define _WINSOCKAPI_
#include <Windows.h>

#include "ethercat.h"

char IOmap[4096];

/* define pointer structure */
#pragma pack(1)
typedef struct {
    uint16 Statusword;
    int8 OpModeDisplay;
    int32 PositionActualValue;
    uint32 DigitalInputs;

} in_step_motor;

typedef struct {
    uint16 Controlword;
    int8 OpMode;
    int32 TargetPosition;
    int32 TargeteVelocity;

} out_step_motor;
#pragma pack()


struct SlaveIO
{
    in_step_motor* input;
    out_step_motor* output;
};


int PDOsetup(uint16 slave)
{
    int retval = 0;
    uint8 zero_map = 0; // Variable to clear the PDO mapping
    uint16 clear_val = 0x0000; // Value to clear the mapping
    uint32 map_object; // Variable to hold the mapping object
    uint16 map_1c12; // Variable to hold the mapping for PDO
   
    //........................................................................................
    // Map RXPOD
    retval += ec_SDOwrite(slave, 0x1600, 0x00, FALSE, sizeof(zero_map), &zero_map, EC_TIMEOUTSAFE);

    // 2. Configure new PDO mapping
    // Control Word
    map_object = 0x60400010;  // 0x6040:0 Control Word, 16 bits
    retval += ec_SDOwrite(slave, 0x1600, 0x01, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);

    // Mode of Operation
    map_object = 0x60600008;  // 0x6060:0 Mode of Operation, 8 bits
    retval += ec_SDOwrite(slave, 0x1600, 0x02, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);

    // Target Position
    map_object = 0x607A0020;  // 0x607A:0 Target Position, 32 bits
    retval += ec_SDOwrite(slave, 0x1600, 0x03, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);

    // Profile Velocity
    map_object = 0x60810020;  // 0x6081:0 Profile Velocity, 32 bits
    retval += ec_SDOwrite(slave, 0x1600, 0x04, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);

    // Set number of mapped objects
    uint8 map_count = 4;
    retval += ec_SDOwrite(slave, 0x1600, 0x00, FALSE, sizeof(map_count), &map_count, EC_TIMEOUTSAFE);

    // 4. Configure RXPDO allocation
    clear_val = 0x0000; // Clear the mapping
    retval += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(clear_val), &clear_val, EC_TIMEOUTSAFE);
    map_1c12 = 0x1600; // Set the mapping to the new PDO
    retval += ec_SDOwrite(slave, 0x1c12, 0x01, FALSE, sizeof(map_1c12), &map_1c12, EC_TIMEOUTSAFE);
    map_1c12 = 0x0001; // Set the mapping index
    retval += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(map_1c12), &map_1c12, EC_TIMEOUTSAFE);

    //........................................................................................
    // Map TXPOD
    // First, clear the TXPDO mapping
    clear_val = 0x0000;
    retval += ec_SDOwrite(slave, 0x1A00, 0x00, FALSE, sizeof(clear_val), &clear_val, EC_TIMEOUTSAFE);

    // Configure TXPDO mapping entries
    // Status Word (0x6041:0, 16 bits)
    retval = 0;
    uint16 map_1c13;
    map_object = 0x60410010;
    retval += ec_SDOwrite(slave, 0x1A00, 0x01, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);

    // Modes of Operation Display (0x6061:0, 8 bits)
    map_object = 0x60610008;
    retval += ec_SDOwrite(slave, 0x1A00, 0x02, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);

    // Position Actual Value (0x6064:0, 32 bits)
    map_object = 0x60640020;
    retval += ec_SDOwrite(slave, 0x1A00, 0x03, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);

    // Digital Inputs (0x60FD:0, 32 bits)
    map_object = 0x60FD0020;
    retval += ec_SDOwrite(slave, 0x1A00, 0x04, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);


    // Set the number of mapped objects (4 objects)
    map_count = 4;
    retval += ec_SDOwrite(slave, 0x1A00, 0x00, FALSE, sizeof(map_count), &map_count, EC_TIMEOUTSAFE);

    // Configure TXPDO assignment
    // First, clear the assignment
    clear_val = 0x0000;
    retval += ec_SDOwrite(slave, 0x1C13, 0x00, FALSE, sizeof(clear_val), &clear_val, EC_TIMEOUTSAFE);

    // Assign TXPDO to 0x1A00
    map_1c13 = 0x1A00;
    retval += ec_SDOwrite(slave, 0x1C13, 0x01, FALSE, sizeof(map_1c13), &map_1c13, EC_TIMEOUTSAFE);

    // Set the number of assigned PDOs (1 PDO)
    map_1c13 = 0x0001;
    retval += ec_SDOwrite(slave, 0x1C13, 0x00, FALSE, sizeof(map_1c13), &map_1c13, EC_TIMEOUTSAFE);

    while (EcatError) printf("%s", ec_elist2string());

    if (retval < 0) {
        printf("TXPDO Mapping failed\n");
        exit(-1);
    }

    printf("slave %d set, retval = %d\n", slave, retval);
    return 1;
}

void slaveinfo(const char* ifname)
{
    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname))
    {
        printf("ec_init on %s succeeded.\n", ifname);
        /* find and auto-config slaves */
        if (ec_config_init(FALSE) > 0)
        {
            printf("%d slaves found and configured.\n", ec_slavecount);
            ec_configdc();
            ec_slave[1].PO2SOconfig = &PDOsetup;

            ec_config_map(&IOmap);

            printf("Slaves mapped, state to SAFE_OP.\n");
            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* send one valid process data to make outputs in slaves happy*/
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);


            /* request OP state for all slaves */
            ec_writestate(0);

            int chk = 40;
            /* wait for all slaves to reach OP state */
            do
            {
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
            ec_readstate();//¶ÁÈ¡×´Ì¬


            if (ec_slave[1].state == EC_STATE_OPERATIONAL) {
                printf("state to OPERATION.\n");
            }
            else {
                printf("ERROR! \n");
                return;
            }

            
            SlaveIO slave_io;

            slave_io.output = (out_step_motor*)ec_slave[1].outputs;
            slave_io.input = (in_step_motor*)ec_slave[1].inputs;

            while (1) {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);

                if ((slave_io.input->Statusword & 0x0008) == 0x0008) {
                    slave_io.output->Controlword = 0x80;
                }
                else if ((slave_io.input->Statusword & 0x0040) == 0x0040) {
                    slave_io.output->Controlword = 0x6;
                }
                else if ((slave_io.input->Statusword & 0x003F) == 0x0031) {
                    slave_io.output->Controlword = 0x7;
                }
                else if ((slave_io.input->Statusword & 0x003F) == 0x0033) {
                    slave_io.output->Controlword = 0xF;
                }
                else if ((slave_io.input->Statusword & 0x003F) == 0x0037) {

                    slave_io.output->TargetPosition = slave_io.input->PositionActualValue + 1;
                }

                printf("Current position: %d\tTarget Position: %d\tStatus: %x\tMode: %d\n", slave_io.input->PositionActualValue, slave_io.output->TargetPosition, slave_io.input->Statusword, slave_io.input->OpModeDisplay);

            }

        }
        else
        {
            printf("No slaves found!\n");
        }
        printf("End slaveinfo, close socket\n");
        /* stop SOEM, close socket */
        ec_close();
    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n", ifname);
    }
}

char ifbuf[1024];

int main(int argc, char* argv[])
{
    printf("SOEM (Simple Open EtherCAT Master)\nSlaveinfo\n");
    slaveinfo("/Device/NPF_{55D111F3-B365-48D5-A31C-FB2C3DE71960}");

    printf("End program\n");
    return (0);
}
