"use client";

import { useRef, useState } from "react";

type CommandResult = {
  command: string;
  behave: string | null;
  status: "simulated" | "unconfigured";
  message: string;
};

type TranscriptionResult = {
  text: string;
  keywords: string[];
  commands: CommandResult[];
  extraction_error: string | null;
};

export default function Home() {
  const [result, setResult] = useState<TranscriptionResult | null>(null);
  const [error, setError] = useState("");
  const [loading, setLoading] = useState(false);
  const pending = useRef(false);

  async function transcribe() {
    if (pending.current) return;
    pending.current = true;
    setLoading(true);
    setError("");
    setResult(null);
    try {
      const response = await fetch("/api/stt", { method: "POST" });
      const data = await response.json().catch(() => null);
      if (!response.ok) {
        throw new Error(data?.detail || "FastAPI 서버 연결을 확인하세요.");
      }
      if (
        typeof data?.text !== "string" ||
        !Array.isArray(data.keywords) ||
        !data.keywords.every((keyword: unknown) => typeof keyword === "string") ||
        !Array.isArray(data.commands) ||
        !data.commands.every((command: CommandResult | null) =>
          command && typeof command.command === "string" &&
          (typeof command.behave === "string" || command.behave === null) &&
          typeof command.message === "string" &&
          ["simulated", "unconfigured"].includes(command.status)
        ) ||
        (data.extraction_error !== null && typeof data.extraction_error !== "string")
      ) {
        throw new Error("서버 응답에 올바른 변환·추출 결과가 없습니다.");
      }
      setResult(data);
    } catch (error) {
      setError(error instanceof Error ? error.message : "요청에 실패했습니다.");
    } finally {
      pending.current = false;
      setLoading(false);
    }
  }

  return (
    <main className="mx-auto flex min-h-screen max-w-2xl flex-col gap-6 px-6 py-20">
      <h1 className="text-3xl font-bold">음성 → 키워드·명령 추출</h1>
      <p className="text-sm leading-7">
        버튼을 누르고 5초 동안 말해 주세요. FastAPI 서버가 실행 중인 PC의
        마이크로 녹음한 뒤 DB에 등록된 키워드와 명령을 추출합니다.
        명령에 연결된 behave는 UI에서 모의 수행 결과로 표시합니다.
      </p>
      <button
        type="button"
        onClick={transcribe}
        disabled={loading}
        className="self-start rounded-lg bg-blue-600 px-6 py-3 font-semibold text-white disabled:cursor-wait disabled:opacity-50"
      >
        {loading ? "녹음 및 분석 중…" : "5초 녹음 시작"}
      </button>
      {error && <p role="alert" className="text-red-600">{error}</p>}
      <section aria-live="polite" aria-busy={loading} className="rounded-xl border border-gray-300 p-6">
        <h2 className="mb-3 font-semibold">변환 결과</h2>
        <p className="whitespace-pre-wrap leading-7">
          {loading ? "5초 녹음 후 음성을 변환하고 키워드·명령을 추출하고 있습니다…" : result?.text || "녹음을 시작하면 결과가 여기에 표시됩니다."}
        </p>
        {result && (
          <div className="mt-6 flex flex-col gap-6 border-t border-gray-300 pt-6">
            {result.extraction_error ? (
              <p role="alert" className="text-red-600">{result.extraction_error}</p>
            ) : (
              <>
                <div>
                  <h2 className="mb-3 font-semibold">키워드 추출 결과</h2>
                  {result.keywords.length > 0 ? (
                    <ul className="space-y-2">
                      {result.keywords.map((keyword) => (
                        <li key={keyword}>{keyword} 키워드를 추출했습니다.</li>
                      ))}
                    </ul>
                  ) : <p>문장에서 DB에 등록된 키워드를 찾지 못했습니다.</p>}
                </div>
                <div>
                  <h2 className="mb-3 font-semibold">명령 및 동작 수행 결과 (UI 시뮬레이션)</h2>
                  {result.commands.length > 0 ? (
                    <ul className="space-y-3">
                      {result.commands.map((command) => (
                        <li key={`${command.command}:${command.behave}`}>
                          <p>명령 추출: {command.command}</p>
                          <p className={command.status === "unconfigured" ? "text-amber-600" : ""}>
                            {command.message}
                          </p>
                        </li>
                      ))}
                    </ul>
                  ) : <p>문장에서 DB에 등록된 명령을 찾지 못했습니다.</p>}
                  <p className="mt-3 text-sm text-gray-500">동작 수행 메시지는 모의 결과이며 실제 장비를 제어하지 않습니다.</p>
                </div>
              </>
            )}
          </div>
        )}
      </section>
    </main>
  );
}
