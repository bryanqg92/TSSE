/* AUTOGENERATED FILE. DO NOT EDIT. */
#include <string.h>
#include <stdlib.h>
#include <setjmp.h>
#include "cmock.h"
#include "mock_logger.h"

static const char* CMockString_format = "format";
static const char* CMockString_level = "level";
static const char* CMockString_logger_init = "logger_init";
static const char* CMockString_logger_log = "logger_log";

typedef struct _CMOCK_logger_init_CALL_INSTANCE
{
  UNITY_LINE_TYPE LineNumber;
  int CallOrder;
  log_level_t Expected_level;

} CMOCK_logger_init_CALL_INSTANCE;

typedef struct _CMOCK_logger_log_CALL_INSTANCE
{
  UNITY_LINE_TYPE LineNumber;
  int CallOrder;
  log_level_t Expected_level;
  const char* Expected_format;

} CMOCK_logger_log_CALL_INSTANCE;

static struct mock_loggerInstance
{
  char logger_init_IgnoreBool;
  char logger_init_CallbackBool;
  CMOCK_logger_init_CALLBACK logger_init_CallbackFunctionPointer;
  int logger_init_CallbackCalls;
  CMOCK_MEM_INDEX_TYPE logger_init_CallInstance;
  char logger_log_IgnoreBool;
  char logger_log_CallbackBool;
  CMOCK_logger_log_CALLBACK logger_log_CallbackFunctionPointer;
  int logger_log_CallbackCalls;
  CMOCK_MEM_INDEX_TYPE logger_log_CallInstance;
} Mock;

extern int GlobalExpectCount;
extern int GlobalVerifyOrder;

void mock_logger_Verify(void)
{
  UNITY_LINE_TYPE cmock_line = TEST_LINE_NUM;
  CMOCK_MEM_INDEX_TYPE call_instance;
  call_instance = Mock.logger_init_CallInstance;
  if (Mock.logger_init_IgnoreBool)
    call_instance = CMOCK_GUTS_NONE;
  if (CMOCK_GUTS_NONE != call_instance)
  {
    UNITY_SET_DETAIL(CMockString_logger_init);
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledLess);
  }
  if (Mock.logger_init_CallbackFunctionPointer != NULL)
  {
    call_instance = CMOCK_GUTS_NONE;
    (void)call_instance;
  }
  call_instance = Mock.logger_log_CallInstance;
  if (Mock.logger_log_IgnoreBool)
    call_instance = CMOCK_GUTS_NONE;
  if (CMOCK_GUTS_NONE != call_instance)
  {
    UNITY_SET_DETAIL(CMockString_logger_log);
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledLess);
  }
  if (Mock.logger_log_CallbackFunctionPointer != NULL)
  {
    call_instance = CMOCK_GUTS_NONE;
    (void)call_instance;
  }
}

void mock_logger_Init(void)
{
  mock_logger_Destroy();
}

void mock_logger_Destroy(void)
{
  CMock_Guts_MemFreeAll();
  memset(&Mock, 0, sizeof(Mock));
  GlobalExpectCount = 0;
  GlobalVerifyOrder = 0;
}

void logger_init(log_level_t level)
{
  UNITY_LINE_TYPE cmock_line = TEST_LINE_NUM;
  CMOCK_logger_init_CALL_INSTANCE* cmock_call_instance;
  UNITY_SET_DETAIL(CMockString_logger_init);
  cmock_call_instance = (CMOCK_logger_init_CALL_INSTANCE*)CMock_Guts_GetAddressFor(Mock.logger_init_CallInstance);
  Mock.logger_init_CallInstance = CMock_Guts_MemNext(Mock.logger_init_CallInstance);
  if (Mock.logger_init_IgnoreBool)
  {
    UNITY_CLR_DETAILS();
    return;
  }
  if (!Mock.logger_init_CallbackBool &&
      Mock.logger_init_CallbackFunctionPointer != NULL)
  {
    Mock.logger_init_CallbackFunctionPointer(level, Mock.logger_init_CallbackCalls++);
    UNITY_CLR_DETAILS();
    return;
  }
  UNITY_TEST_ASSERT_NOT_NULL(cmock_call_instance, cmock_line, CMockStringCalledMore);
  cmock_line = cmock_call_instance->LineNumber;
  if (cmock_call_instance->CallOrder > ++GlobalVerifyOrder)
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledEarly);
  if (cmock_call_instance->CallOrder < GlobalVerifyOrder)
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledLate);
  {
    UNITY_SET_DETAILS(CMockString_logger_init,CMockString_level);
    UNITY_TEST_ASSERT_EQUAL_MEMORY((void*)(&cmock_call_instance->Expected_level), (void*)(&level), sizeof(log_level_t), cmock_line, CMockStringMismatch);
  }
  if (Mock.logger_init_CallbackFunctionPointer != NULL)
  {
    Mock.logger_init_CallbackFunctionPointer(level, Mock.logger_init_CallbackCalls++);
  }
  UNITY_CLR_DETAILS();
}

void CMockExpectParameters_logger_init(CMOCK_logger_init_CALL_INSTANCE* cmock_call_instance, log_level_t level);
void CMockExpectParameters_logger_init(CMOCK_logger_init_CALL_INSTANCE* cmock_call_instance, log_level_t level)
{
  memcpy((void*)(&cmock_call_instance->Expected_level), (void*)(&level),
         sizeof(log_level_t[sizeof(level) == sizeof(log_level_t) ? 1 : -1])); /* add log_level_t to :treat_as_array if this causes an error */
}

void logger_init_CMockIgnore(void)
{
  Mock.logger_init_IgnoreBool = (char)1;
}

void logger_init_CMockStopIgnore(void)
{
  Mock.logger_init_IgnoreBool = (char)0;
}

void logger_init_CMockExpect(UNITY_LINE_TYPE cmock_line, log_level_t level)
{
  CMOCK_MEM_INDEX_TYPE cmock_guts_index = CMock_Guts_MemNew(sizeof(CMOCK_logger_init_CALL_INSTANCE));
  CMOCK_logger_init_CALL_INSTANCE* cmock_call_instance = (CMOCK_logger_init_CALL_INSTANCE*)CMock_Guts_GetAddressFor(cmock_guts_index);
  UNITY_TEST_ASSERT_NOT_NULL(cmock_call_instance, cmock_line, CMockStringOutOfMemory);
  memset(cmock_call_instance, 0, sizeof(*cmock_call_instance));
  Mock.logger_init_CallInstance = CMock_Guts_MemChain(Mock.logger_init_CallInstance, cmock_guts_index);
  Mock.logger_init_IgnoreBool = (char)0;
  cmock_call_instance->LineNumber = cmock_line;
  cmock_call_instance->CallOrder = ++GlobalExpectCount;
  CMockExpectParameters_logger_init(cmock_call_instance, level);
}

void logger_init_AddCallback(CMOCK_logger_init_CALLBACK Callback)
{
  Mock.logger_init_IgnoreBool = (char)0;
  Mock.logger_init_CallbackBool = (char)1;
  Mock.logger_init_CallbackFunctionPointer = Callback;
}

void logger_init_Stub(CMOCK_logger_init_CALLBACK Callback)
{
  Mock.logger_init_IgnoreBool = (char)0;
  Mock.logger_init_CallbackBool = (char)0;
  Mock.logger_init_CallbackFunctionPointer = Callback;
}

void logger_log(log_level_t level, const char* format, ...)
{
  UNITY_LINE_TYPE cmock_line = TEST_LINE_NUM;
  CMOCK_logger_log_CALL_INSTANCE* cmock_call_instance;
  UNITY_SET_DETAIL(CMockString_logger_log);
  cmock_call_instance = (CMOCK_logger_log_CALL_INSTANCE*)CMock_Guts_GetAddressFor(Mock.logger_log_CallInstance);
  Mock.logger_log_CallInstance = CMock_Guts_MemNext(Mock.logger_log_CallInstance);
  if (Mock.logger_log_IgnoreBool)
  {
    UNITY_CLR_DETAILS();
    return;
  }
  if (!Mock.logger_log_CallbackBool &&
      Mock.logger_log_CallbackFunctionPointer != NULL)
  {
    Mock.logger_log_CallbackFunctionPointer(level, format, Mock.logger_log_CallbackCalls++);
    UNITY_CLR_DETAILS();
    return;
  }
  UNITY_TEST_ASSERT_NOT_NULL(cmock_call_instance, cmock_line, CMockStringCalledMore);
  cmock_line = cmock_call_instance->LineNumber;
  if (cmock_call_instance->CallOrder > ++GlobalVerifyOrder)
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledEarly);
  if (cmock_call_instance->CallOrder < GlobalVerifyOrder)
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledLate);
  {
    UNITY_SET_DETAILS(CMockString_logger_log,CMockString_level);
    UNITY_TEST_ASSERT_EQUAL_MEMORY((void*)(&cmock_call_instance->Expected_level), (void*)(&level), sizeof(log_level_t), cmock_line, CMockStringMismatch);
  }
  {
    UNITY_SET_DETAILS(CMockString_logger_log,CMockString_format);
    UNITY_TEST_ASSERT_EQUAL_STRING(cmock_call_instance->Expected_format, format, cmock_line, CMockStringMismatch);
  }
  if (Mock.logger_log_CallbackFunctionPointer != NULL)
  {
    Mock.logger_log_CallbackFunctionPointer(level, format, Mock.logger_log_CallbackCalls++);
  }
  UNITY_CLR_DETAILS();
}

void CMockExpectParameters_logger_log(CMOCK_logger_log_CALL_INSTANCE* cmock_call_instance, log_level_t level, const char* format);
void CMockExpectParameters_logger_log(CMOCK_logger_log_CALL_INSTANCE* cmock_call_instance, log_level_t level, const char* format)
{
  memcpy((void*)(&cmock_call_instance->Expected_level), (void*)(&level),
         sizeof(log_level_t[sizeof(level) == sizeof(log_level_t) ? 1 : -1])); /* add log_level_t to :treat_as_array if this causes an error */
  cmock_call_instance->Expected_format = format;
}

void logger_log_CMockIgnore(void)
{
  Mock.logger_log_IgnoreBool = (char)1;
}

void logger_log_CMockStopIgnore(void)
{
  Mock.logger_log_IgnoreBool = (char)0;
}

void logger_log_CMockExpect(UNITY_LINE_TYPE cmock_line, log_level_t level, const char* format)
{
  CMOCK_MEM_INDEX_TYPE cmock_guts_index = CMock_Guts_MemNew(sizeof(CMOCK_logger_log_CALL_INSTANCE));
  CMOCK_logger_log_CALL_INSTANCE* cmock_call_instance = (CMOCK_logger_log_CALL_INSTANCE*)CMock_Guts_GetAddressFor(cmock_guts_index);
  UNITY_TEST_ASSERT_NOT_NULL(cmock_call_instance, cmock_line, CMockStringOutOfMemory);
  memset(cmock_call_instance, 0, sizeof(*cmock_call_instance));
  Mock.logger_log_CallInstance = CMock_Guts_MemChain(Mock.logger_log_CallInstance, cmock_guts_index);
  Mock.logger_log_IgnoreBool = (char)0;
  cmock_call_instance->LineNumber = cmock_line;
  cmock_call_instance->CallOrder = ++GlobalExpectCount;
  CMockExpectParameters_logger_log(cmock_call_instance, level, format);
}

void logger_log_AddCallback(CMOCK_logger_log_CALLBACK Callback)
{
  Mock.logger_log_IgnoreBool = (char)0;
  Mock.logger_log_CallbackBool = (char)1;
  Mock.logger_log_CallbackFunctionPointer = Callback;
}

void logger_log_Stub(CMOCK_logger_log_CALLBACK Callback)
{
  Mock.logger_log_IgnoreBool = (char)0;
  Mock.logger_log_CallbackBool = (char)0;
  Mock.logger_log_CallbackFunctionPointer = Callback;
}

