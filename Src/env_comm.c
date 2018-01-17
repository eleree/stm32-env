#include "search.h"
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"


extern struct hsearch_data env_htab;
/* Emport the environment and generate CRC for it. */
int env_export(env_t *env_out)
{
	char *res;
	ssize_t	len;
	int ret;

	res = (char *)env_out->data;
	len = hexport_r(&env_htab, '\0', 0, &res, ENV_SIZE, 0, NULL);
	if (len < 0) {
		printf("Cannot export environment\n");
		return 1;
	}

	/* Encrypt the env if desired. */
	//ret = env_aes_cbc_crypt(env_out, 1);
	//if (ret)
	//return ret;

	//env_out->crc = crc32(0, env_out->data, ENV_SIZE);

	return 0;
}

int do_env_set(int flag, int argc, char * const argv[])
{
	int   i, len;
	char  *name, *value, *s;
	ENTRY e, *ep;
	int env_flag = H_INTERACTIVE;

	//debug("Initial value for argc=%d\n", argc);
	while (argc > 1 && **(argv + 1) == '-') {
		char *arg = *++argv;

		--argc;
		while (*++arg) {
			switch (*arg) {
			case 'f':		/* force */
				env_flag |= H_FORCE;
				break;
			default:
				return 0;
			}
		}
	}
	//debug("Final value for argc=%d\n", argc);
	name = argv[1];
	value = argv[2];

	if (strchr(name, '=')) {
		printf("## Error: illegal character '='"
		       "in variable name \"%s\"\n", name);
		return 1;
	}

	//env_id++;

	/* Delete only ? */
	if (argc < 3 || argv[2] == NULL) {
		int rc = hdelete_r(name, &env_htab, env_flag);
		return !rc;
	}

	/*
	 * Insert / replace new value
	 */
	for (i = 2, len = 0; i < argc; ++i)
		len += strlen(argv[i]) + 1;

	value = pvPortMalloc(len);
	if (value == NULL) {
		printf("## Can't malloc %d bytes\n", len);
		return 1;
	}
	for (i = 2, s = value; i < argc; ++i) {
		char *v = argv[i];

		while ((*s++ = *v++) != '\0')
			;
		*(s - 1) = ' ';
	}
	if (s != value)
		*--s = '\0';

	e.key	= name;
	e.data	= value;
	hsearch_r(e, ENTER, &ep, &env_htab, env_flag);
	vPortFree(value);
	if (!ep) {
		printf("## Error inserting \"%s\" variable\n",name);
		return 1;
	}

	return 0;
}

int env_set(const char *varname, const char *varvalue)
{
		const char * const argv[4] = { "setenv", varname, varvalue, NULL };


	if (varvalue == NULL || varvalue[0] == '\0')
		return do_env_set(0, 2, (char * const *)argv);
	else
		return do_env_set(0, 3, (char * const *)argv);
}
