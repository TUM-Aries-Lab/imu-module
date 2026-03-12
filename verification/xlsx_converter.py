def convert_xlsx_to_csv(input_xlsx_path: Path, output_csv_path: Path|None = None ) -> Path:
    """Convert an Excel file to CSV format.

    Reads a single Excel file (.xls or .xlsx) from the testing directory and writes
    its contents to a CSV file with the same filename stem in the same directory.
    This is useful for converting test data and measurement recordings to a
    more portable and scriptable format.

    :param Path path: Absolute path to the Excel file.
    :return: Path to the newly created CSV file with the same name as the input file but with .csv extension.
    :rtype: Path
    """
    if not input_xlsx_path.exists():
        raise FileNotFoundError(f"File not found: {input_xlsx_path}")

    if output_csv_path is None:
        output_csv_path = input_xlsx_path.with_suffix(".csv")

    logger.info(f"Reading Excel file: {input_xlsx_path}")
    data: pd.DataFrame = pd.read_excel(input_xlsx_path)

    logger.info(f"Writing CSV file: {output_csv_path}")
    data.to_csv(output_csv_path, index=False)

    return output_csv_path
