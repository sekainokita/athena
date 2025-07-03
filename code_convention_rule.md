# ATHENA Project Coding Conventions

## Overview
This document defines the coding style guidelines for the ATHENA (KETI SDV Software Defined Vehicle) project.

## 1. Comments

### 1.1 Language Rules
- **All comments must be written in English only.**
- Korean comments are not allowed.

### 1.2 Comment Style
- Single line comments: `/* Comment */`
- Multi-line comments: Use `/* ... */` blocks
- Prefer `/* */` style over `//` style

## 2. Variable Naming

### 2.1 Hungarian Notation Usage
- **All variables must use Hungarian notation.**
- Do not use underscores (`_`).

### 2.2 Prefix Rules

| Prefix | Meaning              | Type                        | Example                        |
|--------|----------------------|-----------------------------|--------------------------------|
| `n`    | Number (Integer)     | `int`, `int32_t`           | `nClientCount`, `nReturnValue` |
| `un`   | Unsigned Number      | `uint32_t`, `unsigned int` | `unDeviceId`, `unPortNumber`   |
| `h`    | Handle               | Handles, pointers          | `hClientMutex`, `hListenSocket`|
| `an`   | Array of Numbers     | Number arrays              | `anClientSockets[MAX_CLIENTS]` |
| `ach`  | Array of Characters  | Character arrays           | `achMsgBuffer[MAX_LINE]`       |
| `sz`   | Size                 | `size_t`                   | `szDataLength`, `szWriteLength`|
| `pv`   | Pointer to Void      | `void*`                    | `pvUserData`, `pvBuffer`       |
| `pn`   | Pointer to Number    | `int*`                     | `pnClientSocket`               |
| `pst`  | Pointer to Struct    | Struct pointers            | `pstMsgManager`                |
| `b`    | Boolean              | `bool`                     | `bIsConnected`, `bProcessComplete`|
| `e`    | Enumeration          | `enum`                     | `eFileType`, `eCommType`       |
| `s_`   | Static Variable      | Static variables           | `s_stMsgManagerTx`             |

### 2.3 Variable Name Examples

```c
// GOOD: Hungarian notation
int nClientCount = 0;
uint8_t achLineBuffer[MAX_BUFFER_SIZE];
bool bIsConnected = FALSE;
pthread_mutex_t hClientMutex;
size_t szDataLength = 0;

// BAD: Snake case or unclear naming
int client_count = 0;
char line_buffer[MAX_BUFFER_SIZE];
bool connected = false;
pthread_mutex_t client_mutex;
size_t data_len = 0;
```

## 3. Data Types

### 3.1 Use Explicit Size Types
- Use `uint8_t` instead of `unsigned char`
- Prefer standard size types

```c
// GOOD: Explicit size types
uint8_t achBuffer[MAX_SIZE];
uint32_t unDeviceId;
int32_t nReturnCode;

// BAD: Non-explicit types
unsigned char buffer[MAX_SIZE];
unsigned int device_id;
int return_code;
```

## 4. Function Structure

### 4.1 Variable Declaration Position
- **All variables must be declared at the beginning of the function.**
- Do not declare variables where they are used.

```c
// GOOD: All variables declared at the top
int32_t SomeFunction(void)
{
    int32_t nRet = FRAMEWORK_ERROR;
    int32_t nLoopCounter = 0;
    size_t szDataLength = 0;
    uint8_t achBuffer[MAX_SIZE];
    bool bProcessComplete = FALSE;
    
    /* Function logic here */
    for (nLoopCounter = 0; nLoopCounter < MAX_COUNT; nLoopCounter++)
    {
        /* ... */
    }
    
    nRet = FRAMEWORK_OK;
    return nRet;
}

// BAD: Variables declared where used
int32_t SomeFunction(void)
{
    for (int i = 0; i < MAX_COUNT; i++)  // BAD: Declaration in loop
    {
        size_t len = strlen(buffer);      // BAD: Declaration in middle
        /* ... */
    }
}
```

### 4.2 Single Return Rule
- **There should be only one return statement per function.**
- Use `goto EXIT` pattern for error handling.

```c
// GOOD: Single return with goto EXIT
int32_t SomeFunction(void)
{
    int32_t nRet = FRAMEWORK_ERROR;
    int32_t nResult = 0;
    
    nResult = SomeOperation();
    if (nResult < 0)
    {
        PrintError("Operation failed");
        nRet = FRAMEWORK_ERROR;
        goto EXIT;
    }
    
    nResult = AnotherOperation();
    if (nResult < 0)
    {
        PrintError("Another operation failed");
        nRet = FRAMEWORK_ERROR;
        goto EXIT;
    }
    
    nRet = FRAMEWORK_OK;
    
EXIT:
    return nRet;
}

// BAD: Multiple returns
int32_t SomeFunction(void)
{
    if (SomeOperation() < 0)
    {
        return FRAMEWORK_ERROR;  // BAD: Early return
    }
    
    if (AnotherOperation() < 0)
    {
        return FRAMEWORK_ERROR;  // BAD: Multiple returns
    }
    
    return FRAMEWORK_OK;
}
```

## 5. Code Formatting

### 5.1 Brace Style
- **Place braces on the next line.**

```c
// GOOD: Braces on new line
if (condition)
{
    /* code here */
}

while (nLoopCounter < MAX_COUNT)
{
    /* code here */
    nLoopCounter++;
}

// BAD: Same line braces
if (condition) {
    /* code here */
}
```

### 5.2 Indentation
- **Use 4 spaces instead of tabs.**
- Maintain consistent indentation.

```c
// GOOD: 4 spaces indentation
if (condition)
{
    if (another_condition)
    {
        DoSomething();
    }
}

// BAD: Tab or inconsistent spacing
if (condition)
{
	if (another_condition)  // TAB used
	{
		DoSomething();
	}
}
```

### 5.3 Whitespace Handling
- **Remove unnecessary whitespace at line endings.**
- **Remove unnecessary whitespace after newlines.**
- Use appropriate spacing around operators.

```c
// GOOD: Proper spacing and no trailing whitespace
nResult = nValue1 + nValue2;
if (nCounter >= MAX_VALUE)
{
    ProcessValue();
}

// BAD: Inconsistent spacing
nResult=nValue1+nValue2;
if(nCounter>=MAX_VALUE)

// BAD: Trailing whitespace after newline
int nValue = 0;    
    /* <- This line has unnecessary whitespace */

// GOOD: No trailing whitespace
int nValue = 0;
/* <- This line has no unnecessary whitespace */
```

## 6. Constants and Macros

### 6.1 No Magic Numbers
- **Define meaningful constants instead of hardcoded numbers.**

```c
// GOOD: Named constants
#define MAX_SAFE_WRITE_LENGTH 500
#define MAX_CLIENTS 100
#define DEFAULT_PORT 8080

if (szWriteLength > MAX_SAFE_WRITE_LENGTH)
{
    szWriteLength = MAX_SAFE_WRITE_LENGTH;
}

// BAD: Magic numbers
if (write_len > 500)  // What does 500 mean?
{
    write_len = 500;
}
```

### 6.2 Constant Naming Rules
- Use all uppercase letters
- Separate words with underscores

```c
#define MAX_BUFFER_SIZE 1024
#define MSG_MANAGER_TIMEOUT 5000
#define CLI_SUCCESS_CODE 0
```

## 7. Error Handling

### 7.1 Error Code Management
- Store error codes in `nRet` variable instead of immediate return
- Use `goto EXIT` pattern for cleanup operations

```c
// GOOD: Proper error handling
int32_t ProcessData(void)
{
    int32_t nRet = FRAMEWORK_ERROR;
    FILE *hFile = NULL;
    uint8_t *puchBuffer = NULL;
    
    hFile = fopen("data.txt", "r");
    if (hFile == NULL)
    {
        PrintError("Failed to open file");
        nRet = FILE_OPEN_ERROR;
        goto EXIT;
    }
    
    puchBuffer = malloc(MAX_BUFFER_SIZE);
    if (puchBuffer == NULL)
    {
        PrintError("Memory allocation failed");
        nRet = MEMORY_ALLOCATION_ERROR;
        goto EXIT;
    }
    
    /* Process data */
    nRet = FRAMEWORK_OK;
    
EXIT:
    if (hFile != NULL)
    {
        fclose(hFile);
    }
    if (puchBuffer != NULL)
    {
        free(puchBuffer);
    }
    return nRet;
}
```

## 8. Function Design Principles

### 8.1 Function Parameters
- Apply Hungarian notation
- Validate input parameters

```c
int32_t ProcessMessage(const char *pchMessage, uint32_t unMessageLength, bool bIsEncrypted)
{
    int32_t nRet = FRAMEWORK_ERROR;
    
    if (pchMessage == NULL || unMessageLength == 0)
    {
        PrintError("Invalid parameters");
        nRet = INVALID_PARAMETER_ERROR;
        goto EXIT;
    }
    
    /* Function implementation */
    nRet = FRAMEWORK_OK;
    
EXIT:
    return nRet;
}
```

## 9. Example: Complete Function Structure

```c
/**
 * @brief Process websocket communication data
 * @param pstWsi Websocket instance pointer
 * @param eCbReason Callback reason enumeration
 * @param pvUser User data pointer
 * @param pvIn Input data pointer
 * @param szLength Data length
 * @return FRAMEWORK_OK on success, error code on failure
 */
static int32_t P_MSG_MANAGER_WebSocketCallback(struct lws *pstWsi, 
                                               enum lws_callback_reasons eCbReason, 
                                               void *pvUser, 
                                               void *pvIn, 
                                               size_t szLength)
{
    int32_t nRet = FRAMEWORK_ERROR;
    int32_t nSystemResult = FRAMEWORK_OK;
    int32_t nWriteResult = 0;
    long lFileSize = 0;
    long lLoopCounter = 0;
    size_t szDataLength = 0;
    size_t szWriteLength = 0;
    uint8_t achLineBuffer[LWS_PRE + MSG_MANAGER_WEBSOCKET_BUF_MAX_LEN];
    bool bProcessComplete = FALSE;

    UNUSED(pvUser);
    UNUSED(pvIn);
    UNUSED(szLength);

    if (pstWsi == NULL)
    {
        PrintError("pstWsi is NULL!");
        nRet = INVALID_PARAMETER_ERROR;
        goto EXIT;
    }

    switch (eCbReason)
    {
        case LWS_CALLBACK_ESTABLISHED:
            /* Handle connection establishment */
            break;
            
        case LWS_CALLBACK_SERVER_WRITEABLE:
            /* Handle data writing */
            if (szWriteLength > MAX_SAFE_WRITE_LENGTH)
            {
                szWriteLength = MAX_SAFE_WRITE_LENGTH;
                achLineBuffer[LWS_PRE + MAX_SAFE_WRITE_LENGTH] = '\0';
                PrintWarn("Truncating line to %d bytes for safety", MAX_SAFE_WRITE_LENGTH);
            }
            break;
            
        default:
            PrintError("Unknown callback reason: %d", eCbReason);
            nRet = UNKNOWN_CALLBACK_ERROR;
            goto EXIT;
    }
    
    nRet = FRAMEWORK_OK;

EXIT:
    return nRet;
}