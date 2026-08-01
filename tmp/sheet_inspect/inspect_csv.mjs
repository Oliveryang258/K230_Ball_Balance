import fs from "node:fs/promises";
import { Workbook } from "@oai/artifact-tool";

const csvPath = "C:/Users/32142/Desktop/ball_run.csv";
const csvText = await fs.readFile(csvPath, "utf8");
const workbook = await Workbook.fromCSV(csvText, { sheetName: "Telemetry" });
const inspection = await workbook.inspect({
  kind: "workbook,sheet,table",
  maxChars: 12000,
  tableMaxRows: 5,
  tableMaxCols: 40,
  tableMaxCellChars: 80,
});
process.stdout.write(inspection.ndjson);
