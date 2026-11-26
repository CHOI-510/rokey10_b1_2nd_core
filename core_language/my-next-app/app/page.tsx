"use client";

import { useRef, useState } from "react";

export default function Home() {
  const [text, setText] = useState("");
  const [error, setError] = useState("");
  const [loading, setLoading] = useState(false);
  const pending = useRef(false);

  async function transcribe() {
    if (pending.current) return;
    pending.current = true;
    setLoading(true);
    setError("");
    setText("");
    try {
      const response = await fetch("/api/stt", { method: "POST" });
      const data = await response.json().catch(() => null);
      if (!response.ok) {
        throw new Error(data?.detail || "FastAPI 서버 연결을 확인하세요.");
      }
      if (typeof data?.text !== "string") {
        throw new Error("서버 응답에 변환 결과가 없습니다.");
      }
      setText(data.text || "인식된 음성이 없습니다. 다시 녹음해 주세요.");
    } catch (error) {
      setError(error instanceof Error ? error.message : "요청에 실패했습니다.");
    } finally {
      pending.current = false;
      setLoading(false);
    }
  }

  return (
    <main className="mx-auto flex min-h-screen max-w-2xl flex-col gap-6 px-6 py-20">
      <h1 className="text-3xl font-bold">음성 → 텍스트</h1>
      <p className="text-sm leading-7">
        버튼을 누르고 5초 동안 말해 주세요. FastAPI 서버가 실행 중인 PC의
        마이크로 녹음한 뒤 변환 결과를 표시합니다.
      </p>
      <button
        type="button"
        onClick={transcribe}
        disabled={loading}
        className="self-start rounded-lg bg-blue-600 px-6 py-3 font-semibold text-white disabled:cursor-wait disabled:opacity-50"
      >
        {loading ? "녹음 및 변환 중…" : "5초 녹음 시작"}
      </button>
      {error && <p role="alert" className="text-red-600">{error}</p>}
      <section aria-live="polite" aria-busy={loading} className="rounded-xl border border-gray-300 p-6">
        <h2 className="mb-3 font-semibold">변환 결과</h2>
        <p className="whitespace-pre-wrap leading-7">
          {loading ? "5초 녹음 후 음성을 텍스트로 변환하고 있습니다…" : text || "녹음을 시작하면 결과가 여기에 표시됩니다."}
        </p>
      </section>
    </main>
  );
}
