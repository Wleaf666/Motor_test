#pragma once

// Existing application task entry points; scheduling and priority are unchanged.
void Task_get_torque(void* argument);
void Task_host_communication(void* argument);
bool Tasks_init();
