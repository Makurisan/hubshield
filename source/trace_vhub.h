#undef TRACE_SYSTEM
#define TRACE_SYSTEM vhub-trace

#undef TRACE_SYSTEM_VAR
#define TRACE_SYSTEM_VAR vhub_trace

#if !defined(_TRACE_EVENT_VHUB_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_EVENT_VHUB_H

#include <linux/tracepoint.h>


DECLARE_EVENT_CLASS(foo_template,

TP_PROTO(const char* foo, int bar),

TP_ARGS(foo, bar),

TP_STRUCT__entry(
  __string(foo, foo)
  __field(int, bar)
),

TP_fast_assign(
  __assign_str(foo, foo);
__entry->bar = bar;
),

TP_printk("foo %s %d", __get_str(foo), __entry->bar)
);

DEFINE_EVENT(foo_template, irq_dtrdy,
  TP_PROTO(const char* foo, int bar),
  TP_ARGS(foo, bar));

#endif

#undef TRACE_INCLUDE_PATH
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_PATH .
/*
 * TRACE_INCLUDE_FILE is not needed if the filename and TRACE_SYSTEM are equal
 */
#define TRACE_INCLUDE_FILE trace_vhub
#include <trace/define_trace.h>
