# CSV Decode Table

## Editing

Edit with Excel, Numbers, or LibreOffice Calc by importing CSV with
separator as “merged spaces”.

Save/Export with space separated CSV.

*Note:* Avoid using spaces (including TAB, newline) in fields.

## Aligning

After saving with Excel/Numbers/Calc, columns may lose alignment, which
can be fixed with Unix command “column -t”.

In VIM under Unix like OSes, it can be opened and aligned with “:%!column -t”

In Unix like OSes, it can also be aligned with command:
```
$ column -t decode-inst.csv | cat > decode-inst.csv
```

## FAQ

* Q: Why using CSV instead of original source code?
A: Raw source code is not friendly for frequently adjustment on decoding
   tables, especially when you want to add new (ro remove old) columns.
   With CSV, that can be more easily handled with GUI table editors.

* Q: Why not using XSLX or ODS format?
A: As well as being manipulated with GUI table programs, aligned CSV can
   also be easily edited with text editors, like source code.  Besides,
   text based CSV can be easily tracked with source code version control
   systems and compared with diff tools.

